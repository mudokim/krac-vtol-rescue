#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
krac_gimbal.approach_node
=========================
"짐벌-큐 접근(gimbal-cued approach)" 협조제어 노드.

전제: 이 노드는 기체가 REP(구조) 지점 상공에 이미 도착해서 짐벌이 직하(-90°)를
보고 있는데도 YOLO/ArUco가 타깃을 못 찾았을 때만 arm 된다(판단은 vtol_fsm 쪽
PHASE_CHECK_TARGET 이 담당). 그래서 이 노드는 60° 전방주시로 "가는 동안" 보는
용도가 아니라, 도착 후 "못 찾았을 때의 탐색"만 담당한다.

시나리오
    ① 짐벌을 하향 60°(sweep_pitch, 직하보다 얕은 각)로 눕히고 동서남북 각 방향을
       순서대로 바라보며(sweep_yaw_deg) 탐색한다.
    ② 어느 한 방향에서 물체를 인식하면
    ③ 짐벌을 90° 직하방으로 "정렬해 가면서" 기체를 물체 바로 위까지 이동시킨다.

원리 (왜 이렇게 하면 되는가)
    - 직하(-90°)에서는 yaw 를 돌려도 항상 발밑의 같은 지점만 보이므로, 지면을
      넓게 훑으려면 pitch 를 얕게(-60°) 눕혀야 yaw 방향마다 다른 지상 지점이 보인다.
    - 짐벌 비주얼 서보가 타깃을 항상 화면 중앙에 유지한다.
      → 그러면 짐벌 pitch = 타깃까지의 실제 내림각(depression angle)이 된다.
        (중앙에 물고 있으면 짐벌이 곧 타깃을 정조준하므로)
    - 기체가 전진해 수평거리 d 가 줄면 내림각은 기하학적으로 60°→90° 로 커진다.
    - 전진속도를 (90° − |pitch|) 에 비례시키면 머리 위(d=0, pitch=−90°)에서
      속도가 0 이 되어 정확히 멈춘다.
    ⇒ "짐벌을 직하로 정렬하며 물체 위로 접근" 이 자연스럽게 완성된다.
    - 접근 방향(vx, vy)은 짐벌 yaw(기체 기준 상대각) 만으로는 못 정한다. 이 프로젝트의
      mavros 속도 셋포인트는 로컬 ENU(월드) 프레임이므로, 반드시 기체 실제 헤딩
      (/mavros/local_position/pose 에서 뽑은 vehicle_yaw)을 더한
      world_yaw = vehicle_yaw + gimbal_yaw 로 변환한 뒤 cos/sin 을 취해야 한다.

상태기계
    IDLE ──(/approach/start = true)──▶ SWEEP ──(한 방향에서 연속 탐지)──▶ APPROACH
      ▲                                                                       │
      │(/approach/start = false, 언제든)                                      ▼
      └────────────────────────────────── OVERHEAD ◀──(직하+중앙 수렴)────────┘

인터페이스
    Sub  /vision/target_error          krac_interfaces/TargetError
    Sub  /gimbal/attitude              geometry_msgs/Vector3Stamped (x=yaw,y=pitch,°)
    Sub  /mavros/local_position/pose   geometry_msgs/PoseStamped (기체 헤딩 보정용)
    Pub  /gimbal/angle_cmd             geometry_msgs/Vector3        (x=yaw,y=pitch,°)
    Pub  /approach/cmd_vel             geometry_msgs/Twist  (vtol_fsm 이 릴레이)
    Pub  /approach/state               std_msgs/String
    Pub  /approach/done                std_msgs/Bool
    Srv  /approach/start               std_srvs/SetBool  (arm/disarm, 안전용)

안전
    - /approach/start(true) 로 arm 하기 전에는 기체 속도를 절대 내보내지 않는다.
    - 타깃 소실/미탐지 시 수평속도 0(호버)로 정지한다.
    - 모든 속도·각도 증분은 파라미터 한계로 클램프된다.
    - 스윕이 계속 실패해도 이 노드는 무한정 반복만 한다(폭주 방지를 위한 전체
      타임아웃은 vtol_fsm 의 PHASE_AWAIT_SEARCH_HANDOFF 가 담당 — 거기서 시간 초과 시
      /approach/start(false) 로 이 노드를 내리고 precision_lander 의 자체 블라인드
      하강 폴백으로 넘긴다).
    ※ 실기 적용 전 SITL 에서 lateral_sign / 게인을 반드시 검증할 것.
"""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, Vector3, Vector3Stamped, PoseStamped
from std_srvs.srv import SetBool

from krac_interfaces.msg import TargetError


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class ApproachNode(Node):
    def __init__(self):
        super().__init__('approach_node')

        # ---------------- 파라미터 ----------------
        self.declare_parameter('control_rate_hz', 15.0)
        # ---- 스윕(SWEEP) 탐색 ----
        self.declare_parameter('sweep_pitch_deg', 60.0)        # 스윕 시 하향각(직하보다 얕게)
        self.declare_parameter('sweep_yaw_deg', [0.0, 90.0, 135.0, -90.0])  # 순회할 방향들(기체 기준 상대각)
        self.declare_parameter('sweep_settle_sec', 1.5)        # 방향 전환 후 짐벌 정착 대기
        self.declare_parameter('sweep_check_sec', 1.0)         # 정착 후 판정 대기 시간
        self.declare_parameter('sweep_detect_frames', 3)       # 그 방향에서 확정 판정할 연속 탐지 프레임
        self.declare_parameter('overhead_pitch_tol_deg', 5.0)  # |pitch|>=90-이값 → 직하 도달
        self.declare_parameter('center_tol', 0.08)             # 정규화 픽셀오차 수렴 임계
        self.declare_parameter('image_width', 1024)
        self.declare_parameter('image_height', 1024)
        # 짐벌 비주얼 서보 게인(정규화 오차[-1,1] → 각도 증분 °/cycle)
        self.declare_parameter('kp_gimbal_pitch', 3.0)
        self.declare_parameter('kp_gimbal_yaw', 3.0)
        self.declare_parameter('gimbal_step_max_deg', 3.0)     # 1주기 최대 각도 변화(스무딩)
        # 기체 접근 속도 법칙
        self.declare_parameter('kv_fwd', 0.04)                 # (90-|pitch|)°당 전진속도
        self.declare_parameter('v_max', 1.5)                   # 최대 수평속도 m/s
        self.declare_parameter('lateral_sign', 1.0)            # 횡방향 부호(프레임 보정)
        # 탐지/안전
        self.declare_parameter('lost_timeout_sec', 1.0)        # 소실 시 정지까지
        self.declare_parameter('overhead_frames', 8)           # OVERHEAD 판정 디바운스
        # 핸드오프
        self.declare_parameter('handoff_precision_lander', False)

        gp = self.get_parameter
        self.rate_hz = max(1.0, float(gp('control_rate_hz').value))
        self.sweep_pitch = abs(float(gp('sweep_pitch_deg').value))
        self.sweep_yaws = [float(v) for v in gp('sweep_yaw_deg').value]
        self.sweep_settle_sec = float(gp('sweep_settle_sec').value)
        self.sweep_check_sec = float(gp('sweep_check_sec').value)
        self.sweep_detect_frames = int(gp('sweep_detect_frames').value)
        self.overhead_tol = float(gp('overhead_pitch_tol_deg').value)
        self.center_tol = float(gp('center_tol').value)
        self.half_w = max(1.0, float(gp('image_width').value) / 2.0)
        self.half_h = max(1.0, float(gp('image_height').value) / 2.0)
        self.kp_gp = float(gp('kp_gimbal_pitch').value)
        self.kp_gy = float(gp('kp_gimbal_yaw').value)
        self.gstep_max = float(gp('gimbal_step_max_deg').value)
        self.kv_fwd = float(gp('kv_fwd').value)
        self.v_max = float(gp('v_max').value)
        self.lat_sign = float(gp('lateral_sign').value)
        self.lost_timeout = float(gp('lost_timeout_sec').value)
        self.overhead_frames = int(gp('overhead_frames').value)
        self.handoff = bool(gp('handoff_precision_lander').value)

        if not self.sweep_yaws:
            self.sweep_yaws = [0.0]

        # ---------------- 상태 ----------------
        self.state = 'IDLE'
        self.armed = False
        self.err = None            # 최신 TargetError
        self.err_stamp = None      # 최신 수신 시각
        self.att_yaw = 0.0         # 짐벌 실제 자세(°)
        self.att_pitch = -90.0
        self.att_stamp = None
        self.vehicle_yaw_rad = 0.0  # 기체 실제 헤딩(ENU, /mavros/local_position/pose 기반)
        self.vehicle_yaw_stamp = None
        self.yaw_cmd = 0.0         # 짐벌 목표각(°)
        self.pitch_cmd = -90.0
        self.detect_count = 0
        self.overhead_count = 0
        self.sweep_idx = 0
        self.sweep_dir_enter = self.get_clock().now()
        self.sweep_cycle_count = 0
        self.state_enter = self.get_clock().now()

        # ---------------- ROS 인터페이스 ----------------
        self.create_subscription(TargetError, '/vision/target_error', self.on_err, 10)
        self.create_subscription(Vector3Stamped, '/gimbal/attitude', self.on_att, 10)
        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.on_pose, qos_profile_sensor_data)
        self.gimbal_pub = self.create_publisher(Vector3, '/gimbal/angle_cmd', 10)
        self.vel_pub = self.create_publisher(Twist, '/approach/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/approach/state', 10)
        self.done_pub = self.create_publisher(Bool, '/approach/done', 10)
        self.create_service(SetBool, '/approach/start', self.on_start)
        self._land_cli = self.create_client(SetBool, '/precision_lander/enable')

        self.create_timer(1.0 / self.rate_hz, self.control_loop)
        self.get_logger().info(
            'approach_node 대기중(IDLE). `ros2 service call /approach/start '
            'std_srvs/srv/SetBool "{data: true}"` 로 시작.')

    # ---------------- 콜백 ----------------
    def on_err(self, msg):
        self.err = msg
        self.err_stamp = self.get_clock().now()

    def on_att(self, msg):
        self.att_yaw = float(msg.vector.x)
        self.att_pitch = float(msg.vector.y)
        self.att_stamp = self.get_clock().now()

    def on_pose(self, msg):
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.vehicle_yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        self.vehicle_yaw_stamp = self.get_clock().now()

    def on_start(self, request, response):
        self.armed = bool(request.data)
        if self.armed:
            self._goto('SWEEP')
            response.message = '접근기동 시작(SWEEP)'
        else:
            self._goto('IDLE')
            self._publish_vel(0.0, 0.0)   # 즉시 정지
            response.message = '접근기동 중단(IDLE)'
        response.success = True
        self.get_logger().info(response.message)
        return response

    # ---------------- 유틸 ----------------
    def _goto(self, state):
        if state != self.state:
            self.state = state
            self.state_enter = self.get_clock().now()
            self.detect_count = 0
            self.overhead_count = 0
            if state == 'SWEEP':
                self.sweep_idx = 0
                self.sweep_dir_enter = self.get_clock().now()
                self.sweep_cycle_count = 0
        sm = String()
        sm.data = self.state
        self.state_pub.publish(sm)

    def _elapsed(self):
        return (self.get_clock().now() - self.state_enter).nanoseconds * 1e-9

    def _detected(self):
        if self.err is None or not self.err.is_detected:
            return False
        if self.err_stamp is None:
            return False
        age = (self.get_clock().now() - self.err_stamp).nanoseconds * 1e-9
        return age <= self.lost_timeout

    def _norm_err(self):
        """정규화 픽셀오차 (ex: +우, ey: +위), 각각 대략 [-1,1]."""
        ex = clamp(self.err.pixel_err_x / self.half_w, -1.0, 1.0)
        ey = clamp(self.err.pixel_err_y / self.half_h, -1.0, 1.0)
        return ex, ey

    def _pitch_now(self):
        """짐벌 실제 pitch(°). 자세 피드백이 신선하면 그걸, 아니면 명령값 사용."""
        if self.att_stamp is not None:
            age = (self.get_clock().now() - self.att_stamp).nanoseconds * 1e-9
            if age <= 1.0:
                return self.att_pitch
        return self.pitch_cmd

    def _send_gimbal(self, yaw_deg, pitch_deg):
        v = Vector3()
        v.x = float(yaw_deg)
        v.y = float(pitch_deg)
        v.z = 0.0
        self.gimbal_pub.publish(v)

    def _publish_vel(self, vx, vy, vz=0.0, wz=0.0):
        t = Twist()
        t.linear.x = float(vx)
        t.linear.y = float(vy)
        t.linear.z = float(vz)
        t.angular.z = float(wz)
        self.vel_pub.publish(t)

    # ---------------- 메인 제어 루프 ----------------
    def control_loop(self):
        if self.state == 'IDLE':
            return  # arm 전에는 기체·짐벌에 아무 명령도 내보내지 않음

        if self.state == 'SWEEP':
            # sweep_yaws 를 순서대로 훑으며, 각 방향에서 sweep_pitch 로 눕혀 탐색.
            # 기체는 정지(호버) — 짐벌만 움직인다.
            self.yaw_cmd = self.sweep_yaws[self.sweep_idx]
            self.pitch_cmd = -self.sweep_pitch
            self._send_gimbal(self.yaw_cmd, self.pitch_cmd)
            self._publish_vel(0.0, 0.0)

            dir_elapsed = (self.get_clock().now() - self.sweep_dir_enter).nanoseconds * 1e-9
            if dir_elapsed < self.sweep_settle_sec:
                return  # 짐벌이 아직 이 방향으로 정착 중 — 판정 보류

            if self._detected():
                self.detect_count += 1
                if self.detect_count >= self.sweep_detect_frames:
                    self.get_logger().info(
                        f'스윕 중 타깃 확보(yaw={self.yaw_cmd:.0f}°) → APPROACH')
                    self._goto('APPROACH')
                return
            else:
                self.detect_count = 0

            if dir_elapsed >= self.sweep_settle_sec + self.sweep_check_sec:
                self.sweep_idx += 1
                if self.sweep_idx >= len(self.sweep_yaws):
                    self.sweep_idx = 0
                    self.sweep_cycle_count += 1
                    self.get_logger().warn(
                        f'스윕 {self.sweep_cycle_count}바퀴째 미탐지 — 계속 반복 '
                        '(전체 타임아웃은 vtol_fsm 이 관리)')
                self.sweep_dir_enter = self.get_clock().now()
                self.detect_count = 0
            return

        if self.state == 'APPROACH':
            self._approach_step()
            return

        if self.state == 'OVERHEAD':
            # 직하 정렬 완료: 수평 정지, 짐벌 −90° 고정, done 발행.
            self._send_gimbal(0.0, -90.0)
            self._publish_vel(0.0, 0.0)
            d = Bool()
            d.data = True
            self.done_pub.publish(d)
            return

    def _approach_step(self):
        # 타깃 소실 → 안전 정지(호버) + 짐벌 현 자세 유지
        if not self._detected():
            self._send_gimbal(self.yaw_cmd, self.pitch_cmd)
            self._publish_vel(0.0, 0.0)
            self.overhead_count = 0
            return

        ex, ey = self._norm_err()

        # ── ① 짐벌 비주얼 서보: 타깃을 화면 중앙에 유지(절대각 증분) ──
        #   ey>0(타깃이 위) → 더 위를 봄(pitch 증가, 덜 음수)
        #   ex>0(타깃이 우) → 우로 팬(yaw 증가)
        dpitch = clamp(self.kp_gp * ey, -self.gstep_max, self.gstep_max)
        dyaw = clamp(self.kp_gy * ex, -self.gstep_max, self.gstep_max)
        self.pitch_cmd = clamp(self.pitch_cmd + dpitch, -90.0, 0.0)
        self.yaw_cmd = clamp(self.yaw_cmd + dyaw, -135.0, 135.0)
        self._send_gimbal(self.yaw_cmd, self.pitch_cmd)

        # ── ② 기체 접근: 전진속도 ∝ (90 − |pitch|), 방향 = 기체헤딩 + 짐벌yaw(월드 ENU) ──
        #   mavros 속도 셋포인트가 로컬 ENU 프레임이므로, 짐벌의 기체-상대 yaw 만으로
        #   방향을 잡으면 기체가 동쪽(ENU 0°)을 보고 있을 때만 맞다. 반드시 기체
        #   실제 헤딩(vehicle_yaw_rad)을 더해 월드 방위각으로 변환해야 한다.
        pitch_now = self._pitch_now()
        remaining = clamp(90.0 - abs(pitch_now), 0.0, 90.0)   # 직하까지 남은 각(°)
        v_fwd = clamp(self.kv_fwd * remaining, 0.0, self.v_max)
        gimbal_yaw_deg = self.att_yaw if self.att_stamp else self.yaw_cmd
        world_yaw_rad = self.vehicle_yaw_rad + math.radians(gimbal_yaw_deg)
        vx = v_fwd * math.cos(world_yaw_rad)                  # 월드 ENU x(동)
        vy = self.lat_sign * v_fwd * math.sin(world_yaw_rad)  # 월드 ENU y(북)
        self._publish_vel(vx, vy, vz=0.0, wz=0.0)             # 고도·기수 유지

        # ── ③ OVERHEAD 판정: 직하(pitch≈-90) + 화면 중앙 수렴 ──
        centered = abs(ex) < self.center_tol and abs(ey) < self.center_tol
        overhead = abs(pitch_now) >= (90.0 - self.overhead_tol)
        if centered and overhead:
            self.overhead_count += 1
            if self.overhead_count >= self.overhead_frames:
                self.get_logger().info('물체 상단 도달(직하 정렬 완료) → OVERHEAD')
                self._goto('OVERHEAD')
                if self.handoff:
                    self._call_handoff()
        else:
            self.overhead_count = 0

    def _call_handoff(self):
        if not self._land_cli.service_is_ready():
            self.get_logger().warn('precision_lander 서비스 미준비 — 핸드오프 생략')
            return
        req = SetBool.Request()
        req.data = True
        self._land_cli.call_async(req)
        self.get_logger().info('precision_lander 로 핸드오프(enable=true)')


def main(args=None):
    rclpy.init(args=args)
    node = ApproachNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node._publish_vel(0.0, 0.0)   # 종료 시 정지
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
