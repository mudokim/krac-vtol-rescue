#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
krac_gimbal.gimbal_node
=======================
SIYI A8 mini 짐벌 "프리셋" 제어 노드.

/gimbal/preset (std_msgs/Int32) 로 1 / 2 / 3 을 보내면 파라미터(yaml)에 정의된
프리셋 각도(yaw, pitch)로 짐벌을 절대각 제어한다.

    1 -> 중립 (전방 수평)
    2 -> 90도  (직하방, pitch -90)
    3 -> 180도 (요청값; A8 mini 물리 한계로 클램프 — README 참고)

원리:
  - Jetson Orin Nano 가 이더넷으로 A8 mini(192.168.144.25:37260)에 SIYI SDK
    UDP 패킷을 직접 보내 절대각(SET_GIMBAL_ATTITUDE, 0x0e)을 지정한다.
  - requestSetAngles() 는 "한 번" 보내는 비블로킹 명령이고, 짐벌 펌웨어가
    자체 안정화로 그 각도를 유지한다. 패킷 손실 대비로 활성 프리셋을
    hold_rate_hz 로 저속 재전송한다.
  - getAttitude() 로 현재 자세를 읽어 /gimbal/attitude 로 퍼블리시한다.

의존 라이브러리: github.com/mzahana/siyi_sdk
    Jetson 에서:  pip install siyi-sdk   (또는 저장소를 PYTHONPATH 에 추가)
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Vector3, Vector3Stamped
from std_srvs.srv import Trigger

try:
    from siyi_sdk import SIYISDK
    _IMPORT_ERR = None
except ImportError as e:  # 빌드 환경엔 없어도 되고, 실행(Jetson)에서만 필요
    SIYISDK = None
    _IMPORT_ERR = e


class GimbalNode(Node):
    def __init__(self):
        super().__init__('gimbal_node')

        # ---------------- 파라미터 ----------------
        self.declare_parameter('ip', '192.168.144.25')
        self.declare_parameter('port', 37260)
        # A8 mini 하드웨어 각도 한계 (SIYI SDK cameras.py 기준)
        self.declare_parameter('pitch_min', -90.0)
        self.declare_parameter('pitch_max', 25.0)
        self.declare_parameter('yaw_min', -135.0)
        self.declare_parameter('yaw_max', 135.0)
        # 활성 프리셋 각도 재전송 주기(패킷 손실 대비). 0 이하면 재전송 끔.
        self.declare_parameter('hold_rate_hz', 2.0)
        self.declare_parameter('attitude_pub_rate_hz', 10.0)
        # 프리셋 = [yaw_deg, pitch_deg]
        self.declare_parameter('preset1', [0.0, 0.0])     # 중립(전방)
        self.declare_parameter('preset2', [0.0, -90.0])   # 90도 = 직하방
        self.declare_parameter('preset3', [0.0, 25.0])    # 180도 요청 -> 물리 최대 상향
        # 기동 직후 자동으로 물고 있을 프리셋(0 이면 안 물림). REP 도착 전까지는
        # 항상 직하(-90°)를 보고 있어야 하므로 기본값을 preset2 로 둔다.
        self.declare_parameter('boot_preset', 2)

        gp = self.get_parameter
        self.presets = {
            1: [float(v) for v in gp('preset1').value],
            2: [float(v) for v in gp('preset2').value],
            3: [float(v) for v in gp('preset3').value],
        }
        self.lim_yaw = (float(gp('yaw_min').value), float(gp('yaw_max').value))
        self.lim_pitch = (float(gp('pitch_min').value), float(gp('pitch_max').value))

        self.target = None  # 현재 유지 중인 (yaw_deg, pitch_deg)
        boot_preset = int(gp('boot_preset').value)
        if boot_preset in self.presets:
            yaw, pitch = self.presets[boot_preset]
            self.target = self._clamp(yaw, pitch)

        # ---------------- SIYI 연결 ----------------
        if SIYISDK is None:
            self.get_logger().error(
                f'siyi_sdk import 실패: {_IMPORT_ERR}. '
                'Jetson 에서 `pip install siyi-sdk` 하거나 '
                'github.com/mzahana/siyi_sdk 를 PYTHONPATH 에 추가하세요.')
            raise SystemExit(1)

        self.cam = SIYISDK(server_ip=gp('ip').value, port=int(gp('port').value))
        self._connect()

        # ---------------- ROS 인터페이스 ----------------
        self.create_subscription(Int32, '/gimbal/preset', self.on_preset, 10)
        # 연속 절대각 입력(yaw=x, pitch=y, 단위 °). 오토트랙/접근기동이 사용.
        self.create_subscription(Vector3, '/gimbal/angle_cmd', self.on_angle_cmd, 10)
        self.att_pub = self.create_publisher(Vector3Stamped, '/gimbal/attitude', 10)
        self.create_service(Trigger, '/gimbal/center', self.on_center)

        hold_rate = float(gp('hold_rate_hz').value)
        if hold_rate > 0.0:
            self.create_timer(1.0 / hold_rate, self.hold_target)
        att_rate = max(0.1, float(gp('attitude_pub_rate_hz').value))
        self.create_timer(1.0 / att_rate, self.publish_attitude)

        self.get_logger().info(
            'gimbal_node 시작. `ros2 topic pub /gimbal/preset std_msgs/msg/Int32 '
            '"{data: 2}"` 로 프리셋(1/2/3) 전송.')

    # ---------------- 연결 ----------------
    def _connect(self):
        try:
            ok = self.cam.connect()
        except Exception as e:
            self.get_logger().warn(f'SIYI 연결 예외: {e}')
            return False
        if ok:
            self.get_logger().info('SIYI 짐벌 연결 성공')
        else:
            self.get_logger().warn('SIYI 짐벌 연결 실패 — 타이머에서 재시도합니다.')
        return ok

    def _ensure_conn(self):
        try:
            if self.cam.isConnected():
                return True
        except Exception:
            pass
        return self._connect()

    # ---------------- 각도 클램프 ----------------
    def _clamp(self, yaw, pitch):
        y = max(self.lim_yaw[0], min(self.lim_yaw[1], float(yaw)))
        p = max(self.lim_pitch[0], min(self.lim_pitch[1], float(pitch)))
        return y, p

    # ---------------- 프리셋 수신 ----------------
    def on_preset(self, msg):
        n = int(msg.data)
        if n not in self.presets:
            self.get_logger().warn(f'알 수 없는 프리셋 번호: {n} (1/2/3 만 지원)')
            return
        yaw, pitch = self.presets[n]
        yaw_c, pitch_c = self._clamp(yaw, pitch)
        if (yaw_c, pitch_c) != (yaw, pitch):
            self.get_logger().warn(
                f'프리셋{n} 각도({yaw},{pitch})가 하드웨어 한계를 벗어나 '
                f'({yaw_c},{pitch_c})로 클램프됨.')
        self.target = (yaw_c, pitch_c)
        self._send_angles(yaw_c, pitch_c)
        self.get_logger().info(f'프리셋{n} 적용 → yaw={yaw_c}°, pitch={pitch_c}°')

    # ---------------- 연속 각도 명령 수신 ----------------
    def on_angle_cmd(self, msg):
        """/gimbal/angle_cmd (Vector3: x=yaw°, y=pitch°) 절대각 연속 제어.
        프리셋과 동일하게 클램프 후 target 으로 설정 → hold 타이머가 유지."""
        yaw_c, pitch_c = self._clamp(msg.x, msg.y)
        self.target = (yaw_c, pitch_c)
        self._send_angles(yaw_c, pitch_c)  # 로그는 생략(고속 스트림)

    def _send_angles(self, yaw, pitch):
        if not self._ensure_conn():
            return
        try:
            # 내부적으로 int(yaw*10), int(pitch*10) 로 SET_GIMBAL_ATTITUDE(0x0e) 전송
            self.cam.requestSetAngles(float(yaw), float(pitch))
        except Exception as e:
            self.get_logger().warn(f'각도 전송 실패: {e}')

    # ---------------- 프리셋 유지(저속 재전송) ----------------
    def hold_target(self):
        if self.target is not None:
            self._send_angles(*self.target)

    # ---------------- 자세 피드백 ----------------
    def publish_attitude(self):
        if not self._ensure_conn():
            return
        try:
            yaw, pitch, roll = self.cam.getAttitude()
        except Exception:
            return
        m = Vector3Stamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'gimbal'
        m.vector.x = float(yaw)
        m.vector.y = float(pitch)
        m.vector.z = float(roll)
        self.att_pub.publish(m)

    # ---------------- 센터 서비스 ----------------
    def on_center(self, request, response):
        self.target = None  # 유지 중이던 프리셋 해제
        try:
            if self._ensure_conn():
                self.cam.requestCenterGimbal()
            response.success = True
            response.message = '짐벌 센터 복귀'
        except Exception as e:
            response.success = False
            response.message = f'센터 실패: {e}'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = GimbalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.cam.disconnect()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
