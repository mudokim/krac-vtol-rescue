#!/usr/bin/env python3
"""
Temporary rescue placeholder v2.

Purpose
-------
Keep the original full mission runnable while the control/vision team replaces
only this file later.

Runtime sequence
----------------
enable=true
  -> publish local hold setpoints continuously
  -> ready=true
  -> CHECK_TARGET: 짐벌 직하(-90도) 유지한 채 YOLO/ArUco 즉시감지 확인
       - 감지됨 -> PRECISION_DESCENT
       - 미감지(약 2초) -> GIMBAL_SWEEP
  -> GIMBAL_SWEEP: 짐벌을 지면 수직선(직하)에서 60도 눕혀 옆을 보게 한 뒤
     동서남북(0/90/180/-90도) 순회 탐색
       - 한 방향에서 발견 -> VISUAL_APPROACH
       - 전체 타임아웃 -> PRECISION_DESCENT(블라인드 폴백)
  -> VISUAL_APPROACH: 짐벌 비주얼서보로 타깃 중앙 유지하며 60->90도로 정렬,
     동시에 (기체헤딩+짐벌yaw) 월드방향으로 전진 접근
       - 직하 정렬+중앙 수렴 -> PRECISION_DESCENT
  -> PRECISION_DESCENT: precision_lander 활성화, 그 속도 명령을 릴레이
       - 착지 조건 도달 -> AUTO.LAND
  -> confirm landing
  -> disarm
  -> MANUAL_GRASP: 착륙 상태에서 짧은변으로 집게를 자동 정렬한 뒤, 사용자가
     scripts/gripper_teleop.py 로 회전/개폐를 손보고 Enter 로 파지를 확정한다.
     (manual_grasp_enable=false 면 이 단계를 건너뛴다)
  -> wait 5 seconds on the ground
  -> pre-stream the original handoff position
  -> enter OFFBOARD and arm
  -> climb back to the original handoff height
  -> confirm stable hover
  -> result="SUCCESS"
  -> keep holding until enable=false

Important:
- MAVROS local pose/velocity/relative-altitude publishers use sensor-data QoS.
- SUCCESS is published only after the vehicle has actually climbed and
  stabilized. The return mission therefore starts from a safe airborne state.
- CHECK_TARGET/GIMBAL_SWEEP 단계는 위치 setpoint(hold)로 호버링하고,
  VISUAL_APPROACH/PRECISION_DESCENT 단계만 속도 setpoint로 전환한다. 위치와
  속도 setpoint를 동시에 쏘면 서로 덮어써서 기체가 튀므로 절대 겹치지 않게 한다.
"""

from __future__ import annotations

import math
from copy import deepcopy
from enum import Enum, auto
from math import hypot
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped, Vector3, Vector3Stamped
from krac_interfaces.msg import TargetError
from mavros_msgs.msg import ExtendedState, State
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, Empty, Float64, Int32, String
from std_srvs.srv import SetBool as SetBoolSrv


MAV_LANDED_STATE_ON_GROUND = 1


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class Phase(Enum):
    IDLE = auto()
    PRESTREAM = auto()
    GIMBAL_SELFTEST = auto()   # (옵션) REP 도착 시 동서남북 시연 스윕 후 nadir 복귀
    CHECK_TARGET = auto()      # 짐벌 직하 유지한 채 즉시감지 확인
    GIMBAL_SWEEP = auto()      # 미감지 시 동서남북 60도 스윕 탐색
    VISUAL_APPROACH = auto()   # 스윕에서 발견 -> 짐벌 정렬하며 접근
    PRECISION_DESCENT = auto()  # precision_lander 핸드오프 하강
    LANDING = auto()
    MANUAL_GRASP = auto()      # 착륙 상태에서 사용자가 키보드로 그리퍼 조작 -> 엔터 확정
    GROUND_WAIT = auto()
    RESTART_PRESTREAM = auto()
    REARM_OFFBOARD = auto()
    CLIMB = auto()
    RESULT_SENT = auto()


class RescueControllerPlaceholder(Node):
    def __init__(self) -> None:
        super().__init__("rescue_controller_placeholder")

        self.declare_parameter("setpoint_rate_hz", 20.0)
        self.declare_parameter("handover_prestream_sec", 1.5)
        self.declare_parameter("restart_prestream_sec", 1.5)
        self.declare_parameter("landing_stable_sec", 1.0)
        self.declare_parameter("ground_wait_sec", 5.0)
        self.declare_parameter("climb_stable_sec", 1.0)
        self.declare_parameter("landing_timeout_sec", 120.0)
        self.declare_parameter("restart_timeout_sec", 30.0)
        self.declare_parameter("climb_timeout_sec", 45.0)
        self.declare_parameter("max_landed_altitude_m", 0.30)
        self.declare_parameter("max_landed_vertical_speed_mps", 0.20)
        self.declare_parameter("climb_z_tolerance_m", 0.35)
        self.declare_parameter("max_stable_vertical_speed_mps", 0.30)
        self.declare_parameter("max_stable_horizontal_speed_mps", 0.50)

        # ---- (옵션) REP 도착 동서남북 시연 스윕 ----
        # 실제 미션 로직에는 영향 없음. 짐벌이 실제로 도는지 눈으로 확인하려고
        # REP 도착 직후 CHECK_TARGET 전에 한 바퀴 시연한다. 환경변수 GIMBAL_SELFTEST
        # 로 켠다(run_sitl_bt.sh 가 파라미터로 전달).
        self.declare_parameter("selftest_sweep_enable", False)
        self.declare_parameter("selftest_sweep_yaw_deg", [0.0, 90.0, 180.0, -90.0])
        self.declare_parameter("selftest_sweep_offnadir_deg", 60.0)
        self.declare_parameter("selftest_dwell_sec", 3.0)

        self.selftest_sweep_enable = bool(self.get_parameter("selftest_sweep_enable").value)
        self.selftest_sweep_yaw = [float(v) for v in self.get_parameter("selftest_sweep_yaw_deg").value] or [0.0]
        self.selftest_sweep_offnadir = abs(float(self.get_parameter("selftest_sweep_offnadir_deg").value))
        self.selftest_dwell_sec = float(self.get_parameter("selftest_dwell_sec").value)

        # ---- CHECK_TARGET / GIMBAL_SWEEP / VISUAL_APPROACH / PRECISION_DESCENT ----
        # 마지막 실탐지가 이보다 오래됐으면 하강 시작/조향 판단에서 타깃을
        # 못 본 것으로 친다. YOLO 추론 간격(0.25초)+지터는 통과하되, 착륙~파지~
        # 재상승 사이클을 건너뛴 유령(수십 초)은 확실히 떨어뜨리는 값.
        self.declare_parameter("live_detect_max_age_sec", 1.0)
        # 2026-07-17: 2.0 이었다. 그 시절엔 트래커의 KF_HOLD 유령이 재시도 때마다
        # 이 창을 0.5초 만에 통과시켜서 값이 실제로 쓰인 적이 없었다. 유령을 막고
        # 나니 이 2초가 처음으로 진짜 예산이 됐는데, 재시도 직후엔 /vision/reset 으로
        # 트래커를 비워둔 상태라 YOLO 첫 실탐지 + 10프레임 연속(0.5초)을 새로 채워야
        # 한다. 2초는 빠듯하고, 놓치면 GIMBAL_SWEEP 이 직하를 안 보기 때문에(전 방향
        # offnadir=60) 바로 아래 있는 바구니를 60초 동안 못 찾고 헛돈다.
        self.declare_parameter("check_target_wait_sec", 5.0)
        self.declare_parameter("check_target_stable_frames", 10)  # ~0.5s @ setpoint_rate_hz
        # 스윕 각도는 "지면 수직선(직하)에서 몇 도 눕혔는가"로 지정한다. 60 이면
        # 직하에서 60도 벌어져 옆(측면)을 본다 -> yaw 를 돌리면 동서남북 주변을
        # 실제로 훑는다. 짐벌 pitch 명령 규약(0=수평전방, -90=직하)과는 다르므로
        # _offnadir_to_pitch_cmd() 로 변환해서 쓴다.
        self.declare_parameter("sweep_offnadir_deg", 60.0)
        self.declare_parameter("sweep_yaw_deg", [0.0, 90.0, 180.0, -90.0])
        # 실측(2026-07-15): 짐벌 pitch 는 중력을 이기며 움직여서 ~30도/s 로 느리다
        # (yaw 는 ~250도/s). 직하에서 60도까지 눕는 데만 ~2초가 걸리는데 settle 이
        # 1.5초였을 때는 조인트가 44도까지밖에 못 간 상태에서 탐지 체크가 시작돼,
        # 정작 원하는 측면 각도로는 보지도 못하고 다음 방향으로 넘어갔다.
        self.declare_parameter("sweep_settle_sec", 3.0)
        self.declare_parameter("sweep_check_sec", 1.0)
        self.declare_parameter("sweep_detect_stable_frames", 6)
        self.declare_parameter("sweep_total_timeout_sec", 60.0)
        self.declare_parameter("approach_timeout_sec", 45.0)
        self.declare_parameter("kp_gimbal_pitch", 3.0)
        self.declare_parameter("kp_gimbal_yaw", 3.0)
        self.declare_parameter("gimbal_step_max_deg", 3.0)
        # model.sdf 의 gimbal_yaw_joint 한계(+-3.1416rad)와 같은 값. 여기서 각도를
        # wrap 하면 안 된다: JointPositionController 는 조인트 위치 PID 라서
        # 179 -> -179 를 최단경로가 아니라 반대로 358도 돌아버린다. 경계에서는
        # 그냥 saturate 시키고, 타깃을 놓치면 스윕 재시도 경로가 받아준다.
        self.declare_parameter("approach_yaw_limit_deg", 180.0)

        # ---- MANUAL_GRASP (착륙 후 키보드로 그리퍼 조작해 트레이를 집는다) ----
        # 각도-틈 실측(model.sdf 의 gripper_body z=-0.11 기준): -60도=211mm(활짝),
        # -45도=177mm, -30도=138mm(트레이 짧은변 137mm 물림), 0도=60mm.
        # 부호 규약은 krac24 gripper_test.py 와 동일: 음수=열림, 양수=닫힘.
        self.declare_parameter("manual_grasp_enable", True)
        self.declare_parameter("gz_model_name", "standard_vtol_0")
        self.declare_parameter("gripper_open_deg", -60.0)
        # 집는 대상은 rescue_box = basket.dae(흰 바구니 + 안쪽 주황 조난자, 한 덩어리
        # 강체). 외곽 238x138x74mm 이므로 **짧은변 138mm** 를 문다.
        # 각도-틈: -60=211mm(최대) / -45=177 / -43=172 / -38=159 / -30=138 / -24=122.
        # 위치 제어기는 힘=p_gain*각도오차 라서 접촉각을 지나 더 안쪽을 명령해야 눌러준다.
        # 138mm 는 -30 이 딱 접촉(간섭 0 = 파지력 0)이므로 그보다 안쪽인 -26 을 쓴다
        # -> 틈 ~127mm, 간섭 11mm. 이 11mm 는 구 박스(짧은변 170mm)에서 -38 로 검증된
        # 간섭량과 같다.
        # 2026-07-16: 구 box.dae(230x170x174, 짧은변 170mm) 시절 값이 -38 이었다.
        # 메시가 basket.dae 로 바뀌며 짧은변이 170->138 로 줄었는데 -38(틈 159mm)을
        # 그대로 두면 21mm 헐렁해서 집게가 허공을 쥔다.
        self.declare_parameter("gripper_grip_deg", -26.0)
        self.declare_parameter("gripper_finger_min_deg", -90.0)
        self.declare_parameter("gripper_finger_max_deg", 30.0)
        self.declare_parameter("manual_grasp_timeout_sec", 600.0)
        # 착륙 상태에서 카메라(base_link 기준 x=+0.2, 지면위 0.14m)로 그리퍼 손끝
        # (x=0.0, 지면위 0.02m)을 보려면 뒤쪽으로 0.233m 떨어진 지점을 봐야 한다.
        # -> 짐벌 yaw 180도(뒤), 직하에서 59도. 직하만 보면 바닥밖에 안 보여서
        # 집게가 박스를 제대로 물었는지 눈으로 확인할 수가 없다.
        self.declare_parameter("grasp_view_yaw_deg", 180.0)
        self.declare_parameter("grasp_view_offnadir_deg", 59.0)
        self.declare_parameter("kv_fwd", 0.04)
        self.declare_parameter("v_max", 1.5)
        self.declare_parameter("lateral_sign", 1.0)
        self.declare_parameter("center_tol", 0.08)
        self.declare_parameter("overhead_pitch_tol_deg", 5.0)
        self.declare_parameter("overhead_stable_frames", 8)
        self.declare_parameter("precision_land_alt_m", 0.30)
        # 실측(2026-07-15): 60초로는 정밀 하강이 못 끝난다. 기준고도 ~4m 에서
        # Approaching 구간 하강속도(-0.175m/s)로만 내려와도 ~23초, 정렬이 흔들려
        # 재수렴하는 시간까지 감안하면 60초는 빠듯해서 매번 AUTO.LAND 폴백으로
        # 넘어갔다. 넉넉히 잡되 무한정 매달리진 않게 한다.
        self.declare_parameter("precision_descent_timeout_sec", 150.0)

        self.live_detect_max_age_sec = float(self.get_parameter("live_detect_max_age_sec").value)
        self.check_target_wait_sec = float(self.get_parameter("check_target_wait_sec").value)
        self.check_target_stable_frames = int(self.get_parameter("check_target_stable_frames").value)
        self.sweep_offnadir_deg = abs(float(self.get_parameter("sweep_offnadir_deg").value))
        self.sweep_yaw_list = [float(v) for v in self.get_parameter("sweep_yaw_deg").value] or [0.0]
        self.sweep_settle_sec = float(self.get_parameter("sweep_settle_sec").value)
        self.sweep_check_sec = float(self.get_parameter("sweep_check_sec").value)
        self.sweep_detect_stable_frames = int(self.get_parameter("sweep_detect_stable_frames").value)
        self.sweep_total_timeout_sec = float(self.get_parameter("sweep_total_timeout_sec").value)
        self.approach_timeout_sec = float(self.get_parameter("approach_timeout_sec").value)
        self.kp_gimbal_pitch = float(self.get_parameter("kp_gimbal_pitch").value)
        self.kp_gimbal_yaw = float(self.get_parameter("kp_gimbal_yaw").value)
        self.gimbal_step_max_deg = float(self.get_parameter("gimbal_step_max_deg").value)
        self.approach_yaw_limit_deg = abs(float(self.get_parameter("approach_yaw_limit_deg").value))
        self.manual_grasp_enable = bool(self.get_parameter("manual_grasp_enable").value)
        self.gz_model_name = str(self.get_parameter("gz_model_name").value)
        self.gripper_open_deg = float(self.get_parameter("gripper_open_deg").value)
        self.gripper_grip_deg = float(self.get_parameter("gripper_grip_deg").value)
        self.gripper_finger_min_deg = float(self.get_parameter("gripper_finger_min_deg").value)
        self.gripper_finger_max_deg = float(self.get_parameter("gripper_finger_max_deg").value)
        self.manual_grasp_timeout_sec = float(self.get_parameter("manual_grasp_timeout_sec").value)
        self.grasp_view_yaw_deg = float(self.get_parameter("grasp_view_yaw_deg").value)
        self.grasp_view_offnadir_deg = float(self.get_parameter("grasp_view_offnadir_deg").value)
        self.kv_fwd = float(self.get_parameter("kv_fwd").value)
        self.v_max = float(self.get_parameter("v_max").value)
        self.lateral_sign = float(self.get_parameter("lateral_sign").value)
        self.center_tol = float(self.get_parameter("center_tol").value)
        self.overhead_pitch_tol_deg = float(self.get_parameter("overhead_pitch_tol_deg").value)
        self.overhead_stable_frames = int(self.get_parameter("overhead_stable_frames").value)
        self.precision_land_alt_m = float(self.get_parameter("precision_land_alt_m").value)
        self.precision_descent_timeout_sec = float(self.get_parameter("precision_descent_timeout_sec").value)

        self.setpoint_rate_hz = max(
            10.0, float(self.get_parameter("setpoint_rate_hz").value)
        )
        self.handover_prestream_sec = max(
            1.0, float(self.get_parameter("handover_prestream_sec").value)
        )
        self.restart_prestream_sec = max(
            1.0, float(self.get_parameter("restart_prestream_sec").value)
        )
        self.landing_stable_sec = max(
            0.5, float(self.get_parameter("landing_stable_sec").value)
        )
        self.ground_wait_sec = max(
            0.0, float(self.get_parameter("ground_wait_sec").value)
        )
        self.climb_stable_sec = max(
            0.5, float(self.get_parameter("climb_stable_sec").value)
        )
        self.landing_timeout_sec = max(
            10.0, float(self.get_parameter("landing_timeout_sec").value)
        )
        self.restart_timeout_sec = max(
            10.0, float(self.get_parameter("restart_timeout_sec").value)
        )
        self.climb_timeout_sec = max(
            10.0, float(self.get_parameter("climb_timeout_sec").value)
        )
        self.max_landed_altitude_m = float(
            self.get_parameter("max_landed_altitude_m").value
        )
        self.max_landed_vertical_speed_mps = float(
            self.get_parameter("max_landed_vertical_speed_mps").value
        )
        self.climb_z_tolerance_m = float(
            self.get_parameter("climb_z_tolerance_m").value
        )
        self.max_stable_vertical_speed_mps = float(
            self.get_parameter("max_stable_vertical_speed_mps").value
        )
        self.max_stable_horizontal_speed_mps = float(
            self.get_parameter("max_stable_horizontal_speed_mps").value
        )

        # BT handshake topics use reliable/default QoS.
        self.enable_sub = self.create_subscription(
            Bool,
            "/krac/rescue_module/enable",
            self._enable_cb,
            10,
        )
        self.ready_pub = self.create_publisher(
            Bool, "/krac/rescue_module/ready", 10
        )
        self.result_pub = self.create_publisher(
            String, "/krac/rescue_module/result", 10
        )

        # MAVROS telemetry uses sensor-data QoS (BEST_EFFORT).
        # The previous placeholder used default RELIABLE subscriptions, so it
        # never received pose/velocity/rel_alt and never sent ready=true.
        self.pose_sub = self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self._pose_cb,
            qos_profile_sensor_data,
        )
        self.velocity_sub = self.create_subscription(
            TwistStamped,
            "/mavros/local_position/velocity_local",
            self._velocity_cb,
            qos_profile_sensor_data,
        )
        self.rel_alt_sub = self.create_subscription(
            Float64,
            "/mavros/global_position/rel_alt",
            self._rel_alt_cb,
            qos_profile_sensor_data,
        )
        self.extended_state_sub = self.create_subscription(
            ExtendedState,
            "/mavros/extended_state",
            self._extended_state_cb,
            qos_profile_sensor_data,
        )
        self.state_sub = self.create_subscription(
            State,
            "/mavros/state",
            self._state_cb,
            qos_profile_sensor_data,
        )

        self.hold_pub = self.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", 10
        )
        self.set_mode_client = self.create_client(
            SetMode, "/mavros/set_mode"
        )
        self.arming_client = self.create_client(
            CommandBool, "/mavros/cmd/arming"
        )

        # ---- 짐벌 / 비전 / precision_lander 인터페이스 ----
        self.target_error_sub = self.create_subscription(
            TargetError, "/vision/target_error", self._target_error_cb, 10
        )
        self.gimbal_att_sub = self.create_subscription(
            Vector3Stamped, "/gimbal/attitude", self._gimbal_att_cb, 10
        )
        self.precision_vel_sub = self.create_subscription(
            Twist, "/precision_lander/cmd_vel", self._precision_vel_cb, 10
        )
        self.vision_reset_pub = self.create_publisher(Empty, "/vision/reset", 10)
        self.gimbal_cmd_pub = self.create_publisher(Vector3, "/gimbal/angle_cmd", 10)
        self.gimbal_preset_pub = self.create_publisher(Int32, "/gimbal/preset", 10)
        self.velocity_pub = self.create_publisher(
            Twist, "/mavros/setpoint_velocity/cmd_vel_unstamped", 10
        )
        self.precision_lander_client = self.create_client(
            SetBoolSrv, "/precision_lander/enable"
        )

        # ---- 그리퍼 (MANUAL_GRASP) ----
        # 서보 토픽은 이 노드만 발행한다. teleop 은 델타/프리셋만 보내서 서로
        # 같은 토픽에 쏘며 싸우지 않게 한다.
        self.servo4_pub = self.create_publisher(
            Float64, "/model/%s/servo_4" % self.gz_model_name, 10
        )
        self.servo5_pub = self.create_publisher(
            Float64, "/model/%s/servo_5" % self.gz_model_name, 10
        )
        self.servo6_pub = self.create_publisher(
            Float64, "/model/%s/servo_6" % self.gz_model_name, 10
        )
        self.gripper_cmd_sub = self.create_subscription(
            Vector3, "/krac/gripper/cmd", self._gripper_cmd_cb, 10
        )
        self.grasp_confirm_sub = self.create_subscription(
            Empty, "/krac/manual_grasp/confirm", self._grasp_confirm_cb, 10
        )
        self.grasp_retry_sub = self.create_subscription(
            Empty, "/krac/manual_grasp/retry", self._grasp_retry_cb, 10
        )

        self.phase = Phase.IDLE
        self.active = False
        self.latest_pose: Optional[PoseStamped] = None
        self.handoff_pose: Optional[PoseStamped] = None
        self.command_pose: Optional[PoseStamped] = None

        # ---- 짐벌 스윕/접근 상태 ----
        self.target_error: Optional[TargetError] = None
        self.target_error_stamp_ns = 0
        # 마지막 '실탐지'(YOLO/ArUco) 수신 시각. KF_HOLD 예측 재발행분은 갱신하지
        # 않는다 - _target_fresh 가 유령과 정상 추적을 가르는 기준이다.
        self.last_live_detect_ns = 0
        self.gimbal_yaw_deg = 0.0
        self.gimbal_pitch_deg = -90.0
        self.gimbal_att_stamp_ns = 0
        self.precision_vel = Twist()
        self.precision_vel_stamp_ns = 0
        self.check_target_detect_count = 0
        self.selftest_idx = 0
        self.selftest_dir_enter_ns = 0

        # ---- 그리퍼 상태 (MANUAL_GRASP) ----
        self.gripper_rot_deg = 0.0
        self.gripper_finger_deg = self.gripper_open_deg
        self.grasp_confirmed = False
        self.grasp_help_shown = False
        # ESC 재시도: 못 잡았거나 떨어뜨렸을 때 이 구간만 처음으로 되돌린다.
        # (핸드오버 고도로 다시 올라가 CHECK_TARGET 부터 재시작)
        self.retry_requested = False
        # 착륙 직전엔 카메라가 트레이 바로 위라 YOLO 가 예외 없이 타깃을 놓친다
        # (precision_lander.cpp 주석에도 같은 실측이 있다). 그래서 착륙 시점의
        # target_error 는 w=h=0 이라 짧은변을 못 구한다 - 하강 중 마지막으로
        # 제대로 본 OBB 를 따로 들고 있다가 자동정렬에 쓴다.
        self.last_obb_target: Optional[TargetError] = None

        self.sweep_idx = 0
        self.sweep_started_ns = 0
        self.sweep_dir_enter_ns = 0
        self.approach_yaw_cmd_deg = 0.0
        self.approach_pitch_cmd_deg = -60.0
        self.approach_overhead_count = 0

        self.relative_altitude_m = float("inf")
        self.vertical_speed_mps = float("inf")
        self.horizontal_speed_mps = float("inf")
        self.landed_state = 0
        self.current_mode = ""
        self.armed = False

        self.phase_started_ns = 0
        self.stable_started_ns = 0
        self.last_mode_request_ns = 0
        self.last_arm_request_ns = 0
        self.last_disarm_request_ns = 0
        self.last_diag_ns = 0
        self.phase_setpoint_count = 0

        self.timer = self.create_timer(
            1.0 / self.setpoint_rate_hz, self._tick
        )

        self.get_logger().info(
            "Rescue placeholder v2 ready: land -> wait 5 s -> "
            "OFFBOARD climb -> stable hover -> SUCCESS."
        )

    def _now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def _elapsed_sec(self, started_ns: int) -> float:
        if started_ns <= 0:
            return 0.0
        return (self._now_ns() - started_ns) / 1e9

    def _set_phase(self, phase: Phase) -> None:
        self.phase = phase
        self.phase_started_ns = self._now_ns()
        self.stable_started_ns = 0
        self.phase_setpoint_count = 0

    def _enable_cb(self, msg: Bool) -> None:
        if msg.data:
            if not self.active:
                self._start_request()
            return

        if self.active:
            self.get_logger().info(
                "enable=false received: stopping placeholder setpoints."
            )
        self._reset_idle()

    def _pose_cb(self, msg: PoseStamped) -> None:
        self.latest_pose = msg

    def _velocity_cb(self, msg: TwistStamped) -> None:
        self.vertical_speed_mps = msg.twist.linear.z
        self.horizontal_speed_mps = hypot(
            msg.twist.linear.x, msg.twist.linear.y
        )

    def _rel_alt_cb(self, msg: Float64) -> None:
        self.relative_altitude_m = msg.data

    def _extended_state_cb(self, msg: ExtendedState) -> None:
        self.landed_state = int(msg.landed_state)

    def _state_cb(self, msg: State) -> None:
        self.current_mode = msg.mode
        self.armed = bool(msg.armed)

    def _target_error_cb(self, msg: TargetError) -> None:
        self.target_error = msg
        self.target_error_stamp_ns = self._now_ns()
        live = msg.is_detected and self._target_source_is_live(msg.source)
        if live:
            self.last_live_detect_ns = self._now_ns()
        # 착륙 직전 근접 구간에선 YOLO 가 타깃을 놓쳐 bbox 가 0 으로 온다. 실제로
        # OBB 를 본 마지막 프레임만 따로 남겨서 MANUAL_GRASP 의 짧은변 정렬에 쓴다.
        # KF_HOLD 재발행분도 bbox 를 실어오므로(그 값은 마지막 실탐지의 복사본이다)
        # source 로 걸러야 한다 - 안 그러면 '마지막 실탐지'가 아니라 '마지막 재발행'을
        # 잡게 되고, 재시도 사이클을 건너뛴 옛 OBB 로 집게를 돌린다.
        if live and msg.bbox_width > 0.0 and msg.bbox_height > 0.0:
            self.last_obb_target = msg

    def _gimbal_att_cb(self, msg: Vector3Stamped) -> None:
        self.gimbal_yaw_deg = float(msg.vector.x)
        self.gimbal_pitch_deg = float(msg.vector.y)
        self.gimbal_att_stamp_ns = self._now_ns()

    def _precision_vel_cb(self, msg: Twist) -> None:
        self.precision_vel = msg
        self.precision_vel_stamp_ns = self._now_ns()

    def _target_fresh(self, max_age_sec: float = 0.5) -> bool:
        if self.target_error is None or not self.target_error.is_detected:
            return False
        if self.target_error_stamp_ns <= 0:
            return False
        if self._elapsed_sec(self.target_error_stamp_ns) > max_age_sec:
            return False
        # vision_tracker 는 타깃을 놓쳐도 tracking_hold_sec(75초) 동안 칼만 예측값을
        # is_detected=true 로 계속 발행한다(source='KF_HOLD'). 그 메시지는 꼬박꼬박
        # 도착하므로 '도착시각' 기준 age 검사로는 절대 안 걸러진다 - 이전 하강 막판의
        # 픽셀오차를 새 고도에 환산한 유령을 향해 재하강하는 원인이었다.
        #
        # 그렇다고 '최신 메시지의 source 가 KF_HOLD 면 탈락' 으로 판정하면 안 된다:
        # 트래커는 정상 추적 중에도 YOLO 를 inference_interval_sec(0.25초)마다만
        # 돌리고 그 사이 프레임(카메라 30fps)은 전부 KF_HOLD 로 채운다. 즉 건강한
        # 추적의 대다수 메시지가 KF_HOLD 라서, 그걸로 자르면 정상 하강까지 깨진다.
        #
        # 올바른 기준은 '마지막 실탐지가 얼마나 오래됐나' 다. 0.25초 추론 간격과
        # 지터는 통과시키고, 수십 초 묵은 유령만 떨어뜨린다.
        if self.last_live_detect_ns <= 0:
            return False
        return self._elapsed_sec(self.last_live_detect_ns) <= self.live_detect_max_age_sec

    @staticmethod
    def _target_source_is_live(source: str) -> bool:
        # 화이트리스트로 판정한다. hold 재발행분은 source 가 'KF_HOLD' 이고,
        # 비었거나 모르는 값도 실탐지로 인정하지 않는다.
        s = str(source).strip().upper()
        return s.startswith("YOLO") or s.startswith("ARUCO")

    def _norm_target_error(self) -> tuple[float, float]:
        # vision_tracker 는 1024x1024 기준으로 픽셀오차를 발행한다.
        ex = _clamp(self.target_error.pixel_err_x / 512.0, -1.0, 1.0)
        ey = _clamp(self.target_error.pixel_err_y / 512.0, -1.0, 1.0)
        return ex, ey

    def _gripper_cmd_cb(self, msg: Vector3) -> None:
        # teleop 델타/프리셋. 서보 발행은 MANUAL_GRASP 틱에서만 한다.
        preset = int(round(msg.z))
        if preset == 1:
            self.gripper_finger_deg = self.gripper_open_deg
        elif preset == 2:
            self.gripper_finger_deg = self.gripper_grip_deg
        elif preset == 3:
            self.gripper_rot_deg = 0.0
        elif preset == 4:
            self._align_gripper_to_short_side()
        else:
            self.gripper_rot_deg = _clamp(self.gripper_rot_deg + msg.x, -180.0, 180.0)
            self.gripper_finger_deg = _clamp(
                self.gripper_finger_deg + msg.y,
                self.gripper_finger_min_deg,
                self.gripper_finger_max_deg,
            )
        self.get_logger().info(
            "그리퍼: 회전=%.0f도, 손가락=%.0f도 (틈 참고: -60=211mm, -30=138mm)"
            % (self.gripper_rot_deg, self.gripper_finger_deg)
        )

    def _grasp_confirm_cb(self, msg: Empty) -> None:
        if self.phase == Phase.MANUAL_GRASP:
            self.grasp_confirmed = True
            self.get_logger().info("사용자 파지 확정 신호 수신 -> 상승 시퀀스로 진행.")
        else:
            self.get_logger().warn(
                "파지 확정 신호를 받았지만 지금은 MANUAL_GRASP 단계가 아니다(무시)."
            )

    def _grasp_retry_cb(self, msg: Empty) -> None:
        # 파지 실패/낙하 시 이 구간만 재시작. 이미 상승 중이어도 받는다 - 안 잡힌 채
        # 올라간 걸 뒤늦게 알아채는 경우가 실제로 있어서다.
        retryable = (
            Phase.MANUAL_GRASP,
            Phase.GROUND_WAIT,
            Phase.RESTART_PRESTREAM,
            Phase.REARM_OFFBOARD,
            Phase.CLIMB,
        )
        if self.phase not in retryable:
            self.get_logger().warn(
                "재시도 신호를 받았지만 지금 단계(%s)에선 되돌릴 수 없다(무시)." % self.phase.name
            )
            return
        self.retry_requested = True
        self.grasp_confirmed = True  # 상승 경로를 태워서 핸드오버 고도까지 올린다
        self.gripper_finger_deg = self.gripper_open_deg  # 떨어뜨린 걸 다시 물지 않게
        self.get_logger().warn(
            "재시도 요청 -> 집게 열고 핸드오버 고도로 복귀 후 REP 재탐지/재하강."
        )

    def _align_gripper_to_short_side(self, verbose: bool = True) -> None:
        # YOLO OBB(xywhr)의 회전각과 w/h 로 '짧은 변' 방향을 구해 집게 축을 맞춘다.
        # 카메라(직하) 이미지축 -> 기체축 매핑은 precision_lander 와 동일 규약:
        #   body_x(전방) = -pixel_err_y , body_y(좌측) = +pixel_err_x
        # 이 매핑에서 이미지각 t 인 방향은 기체각 t+90도가 된다. 집게 축은 servo_4=0
        # 일 때 기체 Y(=90도)이므로, 결국 servo_4 = (이미지상 짧은변 각도) 가 된다.
        t = self.last_obb_target
        if t is None:
            if verbose:
                self.get_logger().warn(
                    "짧은변 자동정렬: 하강 중 유효한 OBB 를 한 번도 못 봤다 -> 회전 %.0f도 유지. "
                    "키보드 a/d 로 직접 맞추세요." % self.gripper_rot_deg
                )
            return
        theta_deg = math.degrees(t.yaw_err_rad)
        if t.bbox_width > t.bbox_height:
            theta_deg += 90.0  # 짧은 변은 h 축
        while theta_deg > 90.0:
            theta_deg -= 180.0
        while theta_deg < -90.0:
            theta_deg += 180.0
        self.gripper_rot_deg = theta_deg
        if verbose:
            self.get_logger().info(
                "짧은변 자동정렬: OBB각=%.1f도 w=%.0f h=%.0f -> 집게 회전=%.1f도"
                % (math.degrees(t.yaw_err_rad), t.bbox_width, t.bbox_height, theta_deg)
            )

    def _publish_gripper(self) -> None:
        rot = Float64()
        rot.data = math.radians(self.gripper_rot_deg)
        self.servo4_pub.publish(rot)
        fing = Float64()
        fing.data = math.radians(self.gripper_finger_deg)
        self.servo5_pub.publish(fing)
        self.servo6_pub.publish(fing)

    def _offnadir_to_pitch_cmd(self, offnadir_deg: float) -> float:
        # 직하 기준 기울임각 -> 짐벌 pitch 명령. offnadir=0 -> -90(직하),
        # offnadir=60 -> -30(수직선에서 60도 눕혀 옆을 봄), offnadir=90 -> 0(수평).
        return -(90.0 - _clamp(offnadir_deg, 0.0, 90.0))

    def _send_gimbal(self, yaw_deg: float, pitch_deg: float) -> None:
        v = Vector3()
        v.x = float(yaw_deg)
        v.y = float(pitch_deg)
        self.gimbal_cmd_pub.publish(v)

    def _gimbal_pitch_now(self) -> float:
        if self.gimbal_att_stamp_ns > 0 and self._elapsed_sec(self.gimbal_att_stamp_ns) <= 1.0:
            return self.gimbal_pitch_deg
        return self.approach_pitch_cmd_deg

    def _vehicle_yaw_rad(self) -> float:
        if self.latest_pose is None:
            return 0.0
        q = self.latest_pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _publish_body_velocity(self, vx: float, vy: float, vz: float = 0.0, wz: float = 0.0) -> None:
        t = Twist()
        t.linear.x = float(vx)
        t.linear.y = float(vy)
        t.linear.z = float(vz)
        t.angular.z = float(wz)
        self.velocity_pub.publish(t)

    def _set_precision_lander(self, enable: bool) -> None:
        if not self.precision_lander_client.service_is_ready():
            self.get_logger().warn("precision_lander 서비스 미준비 - enable 요청을 건너뜀.")
            return
        req = SetBoolSrv.Request()
        req.data = enable
        self.precision_lander_client.call_async(req)

    def _start_request(self) -> None:
        self.active = True
        self.handoff_pose = (
            deepcopy(self.latest_pose)
            if self.latest_pose is not None
            else None
        )
        self.command_pose = (
            deepcopy(self.handoff_pose)
            if self.handoff_pose is not None
            else None
        )
        self.last_mode_request_ns = 0
        self.last_arm_request_ns = 0
        self.last_disarm_request_ns = 0
        self.check_target_detect_count = 0
        self.selftest_idx = 0
        self.selftest_dir_enter_ns = 0
        self.sweep_idx = 0
        self.sweep_started_ns = 0
        self.sweep_dir_enter_ns = 0
        self.approach_overhead_count = 0
        self._set_phase(Phase.PRESTREAM)
        self.get_logger().info(
            "enable=true: starting external hold pre-stream."
        )

    def _reset_idle(self) -> None:
        self.active = False
        self.phase = Phase.IDLE
        self.handoff_pose = None
        self.command_pose = None
        self.phase_started_ns = 0
        self.stable_started_ns = 0
        self.phase_setpoint_count = 0
        self._set_precision_lander(False)
        self._publish_body_velocity(0.0, 0.0)

    def _publish_setpoint(self) -> None:
        if self.command_pose is None and self.latest_pose is not None:
            self.command_pose = deepcopy(self.latest_pose)

        if self.command_pose is None:
            return

        msg = deepcopy(self.command_pose)
        msg.header.stamp = self.get_clock().now().to_msg()
        if not msg.header.frame_id:
            msg.header.frame_id = "map"
        self.hold_pub.publish(msg)
        self.phase_setpoint_count += 1

    def _publish_ready(self) -> None:
        msg = Bool()
        msg.data = True
        self.ready_pub.publish(msg)
        self.get_logger().info(
            "ready=true: external setpoint stream active."
        )

    def _publish_result(self, result: str) -> None:
        msg = String()
        msg.data = result
        self.result_pub.publish(msg)
        self.get_logger().info("result=%s" % result)
        self._set_phase(Phase.RESULT_SENT)

    def _service_period_elapsed(self, last_ns: int, period: float = 1.0) -> bool:
        return last_ns <= 0 or (self._now_ns() - last_ns) / 1e9 >= period

    def _request_mode(self, mode: str) -> None:
        if not self._service_period_elapsed(self.last_mode_request_ns):
            return
        if not self.set_mode_client.service_is_ready():
            return

        request = SetMode.Request()
        request.base_mode = 0
        request.custom_mode = mode
        self.set_mode_client.call_async(request)
        self.last_mode_request_ns = self._now_ns()
        self.get_logger().info(
            "Requested mode=%s current=%s" % (mode, self.current_mode)
        )

    def _request_arm(self, arm: bool) -> None:
        last_ns = (
            self.last_arm_request_ns if arm
            else self.last_disarm_request_ns
        )
        if not self._service_period_elapsed(last_ns):
            return
        if not self.arming_client.service_is_ready():
            return

        request = CommandBool.Request()
        request.value = arm
        self.arming_client.call_async(request)
        if arm:
            self.last_arm_request_ns = self._now_ns()
        else:
            self.last_disarm_request_ns = self._now_ns()
        self.get_logger().info(
            "Requested arm=%s current=%s"
            % (str(arm).lower(), str(self.armed).lower())
        )

    def _landing_condition(self) -> bool:
        px4_grounded = self.landed_state == MAV_LANDED_STATE_ON_GROUND
        kinematic_grounded = (
            self.relative_altitude_m <= self.max_landed_altitude_m
            and abs(self.vertical_speed_mps)
            <= self.max_landed_vertical_speed_mps
        )
        return px4_grounded or kinematic_grounded

    def _stable_for(self, condition: bool, duration_sec: float) -> bool:
        if not condition:
            self.stable_started_ns = 0
            return False
        if self.stable_started_ns == 0:
            self.stable_started_ns = self._now_ns()
        return self._elapsed_sec(self.stable_started_ns) >= duration_sec

    def _diag(self, text: str) -> None:
        if self._service_period_elapsed(self.last_diag_ns, 2.0):
            self.last_diag_ns = self._now_ns()
            self.get_logger().info(text)

    def _tick(self) -> None:
        if not self.active:
            return

        # Continuous stream for seamless BT <-> external-controller handoff.
        # VISUAL_APPROACH/PRECISION_DESCENT 는 속도 setpoint 를 쓰므로, 위치
        # setpoint(hold)와 동시에 쏘면 서로 덮어써서 기체가 튄다 - 겹치지 않게 한다.
        if self.phase not in (Phase.VISUAL_APPROACH, Phase.PRECISION_DESCENT):
            self._publish_setpoint()

        if self.phase == Phase.PRESTREAM:
            if self.handoff_pose is None and self.latest_pose is not None:
                self.handoff_pose = deepcopy(self.latest_pose)
                self.command_pose = deepcopy(self.latest_pose)

            required = int(
                self.setpoint_rate_hz * self.handover_prestream_sec
            )
            if self.command_pose is not None and self.phase_setpoint_count >= required:
                self._publish_ready()
                # REP 도착: 짐벌 직하(-90도) 명시적으로 재확인(이미 그 상태일 것).
                self._send_gimbal(0.0, -90.0)
                if self.selftest_sweep_enable:
                    self.get_logger().info(
                        "REP 도착: 짐벌 동서남북 시연 스윕 시작(GIMBAL_SELFTEST)."
                    )
                    self.selftest_idx = 0
                    self.selftest_dir_enter_ns = self._now_ns()
                    self._set_phase(Phase.GIMBAL_SELFTEST)
                else:
                    self._set_phase(Phase.CHECK_TARGET)
            else:
                self._diag(
                    "Waiting for valid MAVROS pose/setpoint pre-stream: "
                    "pose=%s samples=%d/%d"
                    % (
                        "yes" if self.command_pose is not None else "no",
                        self.phase_setpoint_count,
                        required,
                    )
                )
            return

        if self.phase == Phase.GIMBAL_SELFTEST:
            # 위치는 hold(위에서 _publish_setpoint 이미 호출). 짐벌만 동서남북 시연.
            yaw_t = self.selftest_sweep_yaw[self.selftest_idx]
            self._send_gimbal(yaw_t, self._offnadir_to_pitch_cmd(self.selftest_sweep_offnadir))
            if self._elapsed_sec(self.selftest_dir_enter_ns) >= self.selftest_dwell_sec:
                self.get_logger().info(
                    "짐벌 시연: 방향 %d/%d (yaw=%.0f도, 직하에서 %.0f도 눕힘) 완료."
                    % (
                        self.selftest_idx + 1,
                        len(self.selftest_sweep_yaw),
                        yaw_t,
                        self.selftest_sweep_offnadir,
                    )
                )
                self.selftest_idx += 1
                self.selftest_dir_enter_ns = self._now_ns()
                if self.selftest_idx >= len(self.selftest_sweep_yaw):
                    self.get_logger().info(
                        "짐벌 동서남북 시연 완료 -> 직하 복귀 후 즉시감지 확인."
                    )
                    self._send_gimbal(0.0, -90.0)
                    self._set_phase(Phase.CHECK_TARGET)
            return

        if self.phase == Phase.CHECK_TARGET:
            self._send_gimbal(0.0, -90.0)
            if self._target_fresh(0.5):
                self.check_target_detect_count += 1
            else:
                self.check_target_detect_count = 0

            if self.check_target_detect_count >= self.check_target_stable_frames:
                self.get_logger().info(
                    "REP 도착 즉시 타깃 감지 -> precision_lander 핸드오프."
                )
                self._set_precision_lander(True)
                self._set_phase(Phase.PRECISION_DESCENT)
                return

            if self._elapsed_sec(self.phase_started_ns) >= self.check_target_wait_sec:
                self.get_logger().warn(
                    "REP 도착했지만 미탐지 -> 짐벌 스윕 탐색 시작(동서남북, 직하에서 %.0f도 눕혀 측면 탐색)."
                    % self.sweep_offnadir_deg
                )
                self.sweep_idx = 0
                self.sweep_started_ns = self._now_ns()
                self.sweep_dir_enter_ns = self._now_ns()
                # CHECK_TARGET 이 '거의 잡을 뻔한' 상태(count=5~9)로 타임아웃하면 그
                # 카운트가 그대로 스윕으로 새어 들어가, 첫 방향에서 단 1프레임만 잡혀도
                # sweep_detect_stable_frames(6)를 넘겨 바로 접근으로 확정된다. 6프레임
                # 확인하라고 둔 게이트가 무력화되므로 구간 경계에서 반드시 0 으로 판다.
                self.check_target_detect_count = 0
                self._set_phase(Phase.GIMBAL_SWEEP)
            return

        if self.phase == Phase.GIMBAL_SWEEP:
            yaw_t = self.sweep_yaw_list[self.sweep_idx]
            self._send_gimbal(yaw_t, self._offnadir_to_pitch_cmd(self.sweep_offnadir_deg))

            dir_elapsed = self._elapsed_sec(self.sweep_dir_enter_ns)
            if dir_elapsed >= self.sweep_settle_sec:
                if self._target_fresh(0.5):
                    self.check_target_detect_count += 1
                    if self.check_target_detect_count >= self.sweep_detect_stable_frames:
                        self.get_logger().info(
                            "스윕 중 타깃 확보(yaw=%.0f도) -> 접근 시작." % yaw_t
                        )
                        self.approach_yaw_cmd_deg = yaw_t
                        self.approach_pitch_cmd_deg = self._offnadir_to_pitch_cmd(
                            self.sweep_offnadir_deg
                        )
                        self.approach_overhead_count = 0
                        self._set_phase(Phase.VISUAL_APPROACH)
                        return
                else:
                    self.check_target_detect_count = 0

                if dir_elapsed >= self.sweep_settle_sec + self.sweep_check_sec:
                    self.sweep_idx = (self.sweep_idx + 1) % len(self.sweep_yaw_list)
                    self.sweep_dir_enter_ns = self._now_ns()
                    self.check_target_detect_count = 0

            if self._elapsed_sec(self.sweep_started_ns) >= self.sweep_total_timeout_sec:
                self.get_logger().warn(
                    "짐벌 스윕 탐색 타임아웃 -> precision_lander 블라인드 하강/재탐색으로 강제 전환."
                )
                self._send_gimbal(0.0, -90.0)
                self._set_precision_lander(True)
                self._set_phase(Phase.PRECISION_DESCENT)
            return

        if self.phase == Phase.VISUAL_APPROACH:
            if not self._target_fresh(0.5):
                self._publish_body_velocity(0.0, 0.0)
                self.approach_overhead_count = 0
                if self._elapsed_sec(self.phase_started_ns) > self.approach_timeout_sec:
                    self.get_logger().warn("접근 중 타깃 소실 지속 -> 스윕 재시도.")
                    self.sweep_idx = 0
                    self.sweep_started_ns = self._now_ns()
                    self.sweep_dir_enter_ns = self._now_ns()
                    self._set_phase(Phase.GIMBAL_SWEEP)
                return

            ex, ey = self._norm_target_error()
            dpitch = _clamp(self.kp_gimbal_pitch * ey, -self.gimbal_step_max_deg, self.gimbal_step_max_deg)
            dyaw = _clamp(self.kp_gimbal_yaw * ex, -self.gimbal_step_max_deg, self.gimbal_step_max_deg)
            self.approach_pitch_cmd_deg = _clamp(self.approach_pitch_cmd_deg + dpitch, -90.0, 0.0)
            self.approach_yaw_cmd_deg = _clamp(
                self.approach_yaw_cmd_deg + dyaw,
                -self.approach_yaw_limit_deg,
                self.approach_yaw_limit_deg,
            )
            self._send_gimbal(self.approach_yaw_cmd_deg, self.approach_pitch_cmd_deg)

            pitch_now = self._gimbal_pitch_now()
            remaining = _clamp(90.0 - abs(pitch_now), 0.0, 90.0)
            v_fwd = _clamp(self.kv_fwd * remaining, 0.0, self.v_max)
            gimbal_yaw_for_math = (
                self.gimbal_yaw_deg if self.gimbal_att_stamp_ns > 0 else self.approach_yaw_cmd_deg
            )
            world_yaw_rad = self._vehicle_yaw_rad() + math.radians(gimbal_yaw_for_math)
            vx = v_fwd * math.cos(world_yaw_rad)
            vy = self.lateral_sign * v_fwd * math.sin(world_yaw_rad)
            self._publish_body_velocity(vx, vy)

            centered = abs(ex) < self.center_tol and abs(ey) < self.center_tol
            overhead = abs(pitch_now) >= (90.0 - self.overhead_pitch_tol_deg)
            if centered and overhead:
                self.approach_overhead_count += 1
                if self.approach_overhead_count >= self.overhead_stable_frames:
                    self.get_logger().info("물체 상단 도달(직하 정렬 완료) -> precision_lander 핸드오프.")
                    self._publish_body_velocity(0.0, 0.0)
                    self._set_precision_lander(True)
                    self._set_phase(Phase.PRECISION_DESCENT)
                    return
            else:
                self.approach_overhead_count = 0

            if self._elapsed_sec(self.phase_started_ns) > self.approach_timeout_sec:
                self.get_logger().warn("접근 타임아웃 -> 현 위치 기준 precision_lander 핸드오프로 강제 전환.")
                self._publish_body_velocity(0.0, 0.0)
                self._set_precision_lander(True)
                self._set_phase(Phase.PRECISION_DESCENT)
            return

        if self.phase == Phase.PRECISION_DESCENT:
            # 내려가는 내내 집게를 박스 짧은변에 맞춰 돌려둔다. 착륙 직전엔 카메라가
            # 너무 붙어서 YOLO 가 타깃을 놓치므로(그때 정렬하려 하면 이미 늦다)
            # 하강 중 계속 갱신해두는 게 중요하다. 손가락은 활짝 열어둔다.
            if self.manual_grasp_enable:
                self._align_gripper_to_short_side(verbose=False)
                self.gripper_finger_deg = self.gripper_open_deg
                self._publish_gripper()

            fresh = (
                self.precision_vel_stamp_ns > 0
                and self._elapsed_sec(self.precision_vel_stamp_ns) < 0.5
            )
            if fresh:
                self._publish_body_velocity(
                    self.precision_vel.linear.x,
                    self.precision_vel.linear.y,
                    self.precision_vel.linear.z,
                    self.precision_vel.angular.z,
                )
            else:
                self._publish_body_velocity(0.0, 0.0)

            if self._landing_condition() or self.relative_altitude_m <= self.precision_land_alt_m:
                self.get_logger().info("정밀착륙 목표 고도 도달 -> AUTO.LAND로 최종 착지.")
                self._set_precision_lander(False)
                self._publish_body_velocity(0.0, 0.0)
                self.last_mode_request_ns = 0
                self._set_phase(Phase.LANDING)
                self._request_mode("AUTO.LAND")
                return

            if self._elapsed_sec(self.phase_started_ns) > self.precision_descent_timeout_sec:
                self.get_logger().warn("정밀착륙 타임아웃 -> 강제 AUTO.LAND.")
                self._set_precision_lander(False)
                self._publish_body_velocity(0.0, 0.0)
                self.last_mode_request_ns = 0
                self._set_phase(Phase.LANDING)
                self._request_mode("AUTO.LAND")
            return

        if self.phase == Phase.LANDING:
            if self.current_mode != "AUTO.LAND":
                self._request_mode("AUTO.LAND")

            if self._stable_for(
                self._landing_condition(),
                self.landing_stable_sec,
            ):
                self.get_logger().info(
                    "Landing confirmed: landed_state=%d alt=%.2f vz=%.2f."
                    % (
                        self.landed_state,
                        self.relative_altitude_m,
                        self.vertical_speed_mps,
                    )
                )
                self._request_arm(False)
                if self.manual_grasp_enable:
                    self._align_gripper_to_short_side()
                    self.gripper_finger_deg = self.gripper_open_deg
                    self.grasp_confirmed = False
                    self.grasp_help_shown = False
                    self._set_phase(Phase.MANUAL_GRASP)
                else:
                    self._send_gimbal(0.0, -90.0)
                    self._set_phase(Phase.GROUND_WAIT)
                return

            if self._elapsed_sec(self.phase_started_ns) > self.landing_timeout_sec:
                self._publish_result("FAILURE:landing_timeout")
            return

        if self.phase == Phase.MANUAL_GRASP:
            # 착륙+disarm 상태. 기체는 스키드로 지면에 앉아 있고 setpoint 는 hold 로
            # 계속 나간다(위에서 _publish_setpoint 이미 호출). 여기서는 그리퍼만 준다.
            if self.armed:
                self._request_arm(False)
            self._publish_gripper()
            # 카메라는 기체 앞쪽(x=+0.2)에 있고 그리퍼는 x=0 이라, 직하만 보면 바닥만
            # 나오고 집게가 화면에 안 들어온다. 뒤쪽 아래로 돌려 파지를 보여준다.
            self._send_gimbal(
                self.grasp_view_yaw_deg,
                self._offnadir_to_pitch_cmd(self.grasp_view_offnadir_deg),
            )

            if not self.grasp_help_shown:
                self.grasp_help_shown = True
                self.get_logger().info(
                    "MANUAL_GRASP: 다른 터미널에서 scripts/gripper_teleop.py 를 실행해 "
                    "집게를 조작하고, 확실히 잡으면 Enter(확정) / 실패하면 ESC(재시도). "
                    "짐벌을 그리퍼 쪽(yaw=%.0f도, 직하에서 %.0f도)으로 돌려놨으니 "
                    "rqt_image_view 로 파지를 확인하세요. (현재 회전=%.0f도, 손가락=%.0f도)"
                    % (
                        self.grasp_view_yaw_deg,
                        self.grasp_view_offnadir_deg,
                        self.gripper_rot_deg,
                        self.gripper_finger_deg,
                    )
                )

            if self.grasp_confirmed:
                self.get_logger().info("파지 확정 -> 짐벌 직하 복귀 후 상승 시퀀스 시작.")
                # MANUAL_GRASP 에서 그리퍼 쪽(yaw=180, 직하에서 59도)으로 돌려놨던 짐벌을
                # 직하로 되돌린다. 지금 보내두면 GROUND_WAIT+상승 동안(짐벌은 ~30도/s)
                # 다 돌아서, 귀환 구간엔 카메라가 지면 수직으로 향한 채 출발한다.
                self._send_gimbal(0.0, -90.0)
                self._set_phase(Phase.GROUND_WAIT)
                return

            if self._elapsed_sec(self.phase_started_ns) > self.manual_grasp_timeout_sec:
                self._publish_result("FAILURE:manual_grasp_timeout")
            else:
                self._diag(
                    "MANUAL_GRASP 대기중: 회전=%.0f도 손가락=%.0f도. Enter 로 확정."
                    % (self.gripper_rot_deg, self.gripper_finger_deg)
                )
            return

        if self.phase == Phase.GROUND_WAIT:
            if self.armed:
                self._request_arm(False)
            # 잡은 트레이를 놓치지 않도록 상승 내내 집게 명령을 계속 유지한다.
            self._publish_gripper()

            if self._elapsed_sec(self.phase_started_ns) >= self.ground_wait_sec:
                if self.handoff_pose is None:
                    self._publish_result("FAILURE:handoff_pose_missing")
                    return

                # Return to the exact position/height at which BT handed over.
                self.command_pose = deepcopy(self.handoff_pose)
                self.last_mode_request_ns = 0
                self.last_arm_request_ns = 0
                self._set_phase(Phase.RESTART_PRESTREAM)
                self.get_logger().info(
                    "Ground wait complete. Pre-streaming climb target "
                    "z=%.2f m."
                    % self.command_pose.pose.position.z
                )
            return

        # 잡은 트레이를 놓치지 않도록 재arm/상승/호버 내내 집게 명령을 유지한다.
        if self.manual_grasp_enable and self.phase in (
            Phase.RESTART_PRESTREAM,
            Phase.REARM_OFFBOARD,
            Phase.CLIMB,
            Phase.RESULT_SENT,
        ):
            self._publish_gripper()

        if self.phase == Phase.RESTART_PRESTREAM:
            required = int(
                self.setpoint_rate_hz * self.restart_prestream_sec
            )
            if self.phase_setpoint_count >= required:
                self.last_mode_request_ns = 0
                self.last_arm_request_ns = 0
                self._set_phase(Phase.REARM_OFFBOARD)
            return

        if self.phase == Phase.REARM_OFFBOARD:
            if self.current_mode != "OFFBOARD":
                self._request_mode("OFFBOARD")
            if not self.armed:
                self._request_arm(True)

            if self.current_mode == "OFFBOARD" and self.armed:
                self._set_phase(Phase.CLIMB)
                self.get_logger().info(
                    "OFFBOARD and armed. Climbing to handoff height."
                )
                return

            if self._elapsed_sec(self.phase_started_ns) > self.restart_timeout_sec:
                self._publish_result("FAILURE:restart_timeout")
            return

        if self.phase == Phase.CLIMB:
            if self.current_mode != "OFFBOARD":
                self._request_mode("OFFBOARD")

            if self.latest_pose is None or self.command_pose is None:
                climb_ok = False
            else:
                z_error = abs(
                    self.latest_pose.pose.position.z
                    - self.command_pose.pose.position.z
                )
                climb_ok = (
                    z_error <= self.climb_z_tolerance_m
                    and abs(self.vertical_speed_mps)
                    <= self.max_stable_vertical_speed_mps
                    and self.horizontal_speed_mps
                    <= self.max_stable_horizontal_speed_mps
                )

            if self._stable_for(climb_ok, self.climb_stable_sec):
                self.get_logger().info(
                    "Climb and hover confirmed: z=%.2f target=%.2f "
                    "vz=%.2f vxy=%.2f."
                    % (
                        self.latest_pose.pose.position.z,
                        self.command_pose.pose.position.z,
                        self.vertical_speed_mps,
                        self.horizontal_speed_mps,
                    )
                )
                if self.retry_requested:
                    # ESC 재시도: 핸드오버 고도까지 올라왔으니 이 구간을 처음부터
                    # 다시 - 짐벌 직하 복귀 + precision_lander 끄고 CHECK_TARGET 으로.
                    self.retry_requested = False
                    self.grasp_confirmed = False
                    self.grasp_help_shown = False
                    self._set_precision_lander(False)
                    self._send_gimbal(0.0, -90.0)
                    self.check_target_detect_count = 0
                    # 트래커의 KF/hold 기억을 버린다. 이걸 안 하면 직전 하강 막판
                    # (고도 ~1m)의 픽셀오차가 tracking_hold_sec(75초) 동안 살아남아,
                    # 여기 핸드오버 고도에서 그 값이 고도 배율만큼 부풀려 환산된
                    # 유령 타깃으로 재하강한다. 예: 고도 1m 에서 150px(=0.14m)로
                    # 놓친 오차가 4m 에서는 0.58m 로 읽혀 [Approaching] 하강이
                    # 걸리고, 내려갈수록 환산오차가 줄어 [Aligned] 까지 승격되며
                    # has_been_aligned_ 가 켜진 채 엉뚱한 곳에 착륙한다.
                    self.last_obb_target = None
                    self.last_live_detect_ns = 0
                    self.vision_reset_pub.publish(Empty())
                    self.get_logger().warn(
                        "재시도: 핸드오버 고도 복귀 완료 -> 비전 트래킹 리셋 후 REP 재탐지/재하강 시작."
                    )
                    self._set_phase(Phase.CHECK_TARGET)
                    return
                self._publish_result("SUCCESS")
                return

            if self._elapsed_sec(self.phase_started_ns) > self.climb_timeout_sec:
                self._publish_result("FAILURE:climb_timeout")
            return

        # RESULT_SENT: keep the final command setpoint alive until BT publishes
        # enable=false after restoring its own hold stream.


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RescueControllerPlaceholder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
