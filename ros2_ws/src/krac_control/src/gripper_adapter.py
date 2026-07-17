#!/usr/bin/env python3
"""
gripper_adapter — 구조 컨트롤러의 Gazebo 서보 토픽을 제어팀 payload_control
서비스로 번역한다. 비행 코드(BT / rescue_controller)는 수정하지 않는다.

들어오는 쪽 (rescue_controller_placeholder.py 가 발행, 단위=라디안):
  /model/<gz_model_name>/servo_4  Float64  집게 회전
  /model/<gz_model_name>/servo_5  Float64  핑거
  /model/<gz_model_name>/servo_6  Float64  핑거 (servo_5 와 항상 같은 값 → 구독 안 함)

나가는 쪽 (제어팀 servo_serial_bridge → ESP32 시리얼):
  /gripper/arm      std_srvs/SetBool                 True=ARM
  /gripper/set      std_srvs/SetBool                 True=OPEN, False=CLOSE
  /gripper/set_yaw  mission_interface/SetServoAngle  0~180도

--------------------------------------------------------------------------
반드시 지켜야 하는 제약 (전부 상대 펌웨어/브리지 코드에서 확인한 것):

1) 서보 토픽은 구조 단계 동안 매 tick 발행된다. 상태가 실제로 바뀔 때만 서비스를
   호출해야 한다. 매번 부르면 시리얼 트랜잭션이 밀려서 못 따라온다.

2) 펌웨어 setYaw() 는 10단계 x delay(300ms) = 약 3초간 아두이노 loop() 를 통째로
   막는다(거리와 무관하게 항상. 단 목표각==현재각이면 즉시 반환). 브리지의
   transact() timeout_sec 기본값이 4.0 초라 여유가 1 초뿐이다. 그래서
     - 반드시 call_async. 이 노드는 rescue_controller 와 별도 프로세스라 여기서
       블로킹해도 비행 쪽 setpoint 에는 영향이 없지만, rclpy 는 단일 스레드
       실행기의 콜백 안에서 동기 call() 을 하면 응답을 처리할 수 없어 데드락에
       빠진다. 그리고 3초씩 막히면 서보 토픽이 밀린다.
     - 앞선 호출이 끝나기 전에는 새 yaw 를 보내지 않는다 (in-flight 가드).
       버린 값은 다음 tick 에 최신값으로 다시 들어온다.

3) 펌웨어는 ARM 되기 전 모든 SET 을 ERR,NOT_ARMED 로 거부한다. 브리지의 auto_arm
   기본값은 False 이므로 여기서 시작 시 1회 ARM 한다. 또한 브리지의 ensure_armed()
   는 자기 미러(self.armed)만 보고 판단하므로, ESP32 만 리셋되면(펌웨어 setup() 이
   armed=false 로 시작) 브리지는 계속 armed 라 믿고 재-ARM 하지 않는다 → 모든
   명령이 조용히 씹힌다. 그래서 응답에서 NOT_ARMED 를 보면 여기서 재-ARM 한다.

--------------------------------------------------------------------------
각도 규약 (주의):
  rescue_controller 의 핑거각은
      gripper_open_deg = -60.0  (활짝 열림)
      gripper_grip_deg = -26.0  (파지)
  로 **둘 다 음수**다. 소스에 있는 "음수=열림, 양수=닫힘" 주석은 이 값들과 맞지
  않으니 임계를 0 으로 잡으면 안 된다(파지할 때마다 열려버린다). 두 값의 중점인
  -43 도를 기본 임계로 쓰고, finger <= 임계 이면 OPEN 으로 본다.

  회전각은 rot=0 이 중립이고, 펌웨어 yaw 홈은 90 도다 → yaw = 90 + rot.
  실제 서보 회전 방향은 하드웨어 장착에 따라 반대일 수 있다. 뒤집혔으면
  yaw_invert:=true 로 띄운다.
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_srvs.srv import SetBool

from mission_interface.srv import SetServoAngle


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


class GripperAdapter(Node):
    def __init__(self):
        super().__init__('gripper_adapter')

        self.declare_parameter('gz_model_name', 'standard_vtol_0')
        # rescue_controller 의 gripper_open_deg / gripper_grip_deg 중점.
        # 두 값을 바꾸면 이 임계도 같이 바꿔야 한다.
        self.declare_parameter('finger_open_threshold_deg', -43.0)
        self.declare_parameter('yaw_center_deg', 90.0)
        self.declare_parameter('yaw_invert', False)
        # 이보다 작게 변한 회전은 무시한다. 펌웨어 yaw 가 3초 블로킹이라
        # 미세 떨림까지 따라가면 시리얼이 계속 막힌다.
        self.declare_parameter('yaw_deadband_deg', 3.0)
        self.declare_parameter('arm_on_start', True)
        self.declare_parameter('service_wait_sec', 10.0)

        self.model = self.get_parameter('gz_model_name').value
        self.finger_threshold = float(
            self.get_parameter('finger_open_threshold_deg').value)
        self.yaw_center = float(self.get_parameter('yaw_center_deg').value)
        self.yaw_invert = bool(self.get_parameter('yaw_invert').value)
        self.yaw_deadband = float(self.get_parameter('yaw_deadband_deg').value)
        self.service_wait_sec = float(self.get_parameter('service_wait_sec').value)

        # 마지막으로 '보낸' 상태. None = 아직 한 번도 안 보냄 → 첫 명령은 무조건 보낸다.
        self.sent_open = None
        self.sent_yaw = None
        self.gripper_busy = False
        self.yaw_busy = False

        self.arm_cli = self.create_client(SetBool, '/gripper/arm')
        self.set_cli = self.create_client(SetBool, '/gripper/set')
        self.yaw_cli = self.create_client(SetServoAngle, '/gripper/set_yaw')

        for name, cli in (('/gripper/arm', self.arm_cli),
                          ('/gripper/set', self.set_cli),
                          ('/gripper/set_yaw', self.yaw_cli)):
            if not cli.wait_for_service(timeout_sec=self.service_wait_sec):
                self.get_logger().error(
                    '%s 서비스가 %.0f초 안에 안 뜸. servo_serial_bridge 가 떠 있는지, '
                    'ESP32 가 연결됐는지 확인할 것. (계속 대기하며 동작은 유지)'
                    % (name, self.service_wait_sec))

        if bool(self.get_parameter('arm_on_start').value):
            self._arm()

        self.create_subscription(
            Float64, '/model/%s/servo_5' % self.model, self._finger_cb, 10)
        self.create_subscription(
            Float64, '/model/%s/servo_4' % self.model, self._rot_cb, 10)

        self.get_logger().info(
            'gripper_adapter 시작: model=%s finger_threshold=%.1f도 '
            'yaw_center=%.0f도 yaw_invert=%s'
            % (self.model, self.finger_threshold, self.yaw_center, self.yaw_invert))

    # ------------------------------------------------------------------
    def _arm(self):
        req = SetBool.Request()
        req.data = True
        future = self.arm_cli.call_async(req)

        def done(fut):
            try:
                res = fut.result()
            except Exception as exc:
                self.get_logger().error('ARM 호출 실패: %s' % exc)
                return
            if res.success:
                self.get_logger().info('그리퍼 ARM 완료: %s' % res.message)
            else:
                self.get_logger().error(
                    'ARM 거부됨: %s — 이 상태로는 모든 그리퍼 명령이 '
                    'ERR,NOT_ARMED 로 씹힌다.' % res.message)

        future.add_done_callback(done)

    def _handle_rejection(self, what: str, message: str):
        """SET 거부 처리. NOT_ARMED 면 ESP32 가 리셋된 것이므로 재-ARM 한다."""
        if 'NOT_ARMED' in message.upper():
            self.get_logger().error(
                '%s 거부: %s — ESP32 가 리셋된 것으로 보임. 재-ARM 시도.'
                % (what, message))
            self._arm()
        else:
            self.get_logger().error('%s 거부: %s' % (what, message))

    # ------------------------------------------------------------------
    def _finger_cb(self, msg: Float64):
        deg = math.degrees(msg.data)
        want_open = deg <= self.finger_threshold

        if want_open == self.sent_open:
            return
        if self.gripper_busy:
            return

        self.gripper_busy = True
        self.sent_open = want_open

        req = SetBool.Request()
        req.data = want_open
        future = self.set_cli.call_async(req)

        def done(fut):
            self.gripper_busy = False
            try:
                res = fut.result()
            except Exception as exc:
                self.get_logger().error('그리퍼 호출 실패: %s' % exc)
                self.sent_open = None   # 재시도 허용
                return
            if res.success:
                self.get_logger().info(
                    '그리퍼 %s (핑거각 %.1f도): %s'
                    % ('OPEN' if want_open else 'CLOSE', deg, res.message))
            else:
                self._handle_rejection('그리퍼 명령', res.message)
                self.sent_open = None   # 재시도 허용

        future.add_done_callback(done)

    # ------------------------------------------------------------------
    def _rot_cb(self, msg: Float64):
        rot_deg = math.degrees(msg.data)
        if self.yaw_invert:
            rot_deg = -rot_deg
        yaw = int(round(_clamp(self.yaw_center + rot_deg, 0.0, 180.0)))

        if self.sent_yaw is not None and abs(yaw - self.sent_yaw) < self.yaw_deadband:
            return
        # 펌웨어가 3초 블로킹이라, 앞 호출이 안 끝났으면 그냥 버린다.
        # 다음 tick 에 최신 값으로 다시 들어온다.
        if self.yaw_busy:
            return

        self.yaw_busy = True
        self.sent_yaw = yaw

        req = SetServoAngle.Request()
        req.angle = yaw
        future = self.yaw_cli.call_async(req)

        def done(fut):
            self.yaw_busy = False
            try:
                res = fut.result()
            except Exception as exc:
                self.get_logger().error('yaw 호출 실패: %s' % exc)
                self.sent_yaw = None
                return
            if res.success:
                self.get_logger().info(
                    'yaw %d도 (회전 %.1f도): %s' % (yaw, rot_deg, res.message))
            else:
                self._handle_rejection('yaw 명령', res.message)
                self.sent_yaw = None

        future.add_done_callback(done)


def main(args=None):
    rclpy.init(args=args)
    node = GripperAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
