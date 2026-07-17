#!/usr/bin/env python3
"""Manual gripper teleop for the REP pickup.

Runs in its own terminal (it needs a TTY for raw key reads). The rescue
controller owns the servo topics - this node only sends deltas/presets on
`/krac/gripper/cmd` and the "I've got it" signal on `/krac/manual_grasp/confirm`,
so the two never fight over `/model/<m>/servo_*`.

Angle convention matches krac24's gripper_test.py and the model geometry:
negative finger angle opens, positive closes. Measured gap vs finger angle
(survivor_tray short side is 137mm):
    -60deg -> 211mm (wide open)   -45deg -> 177mm
    -30deg -> 138mm (grips tray)    0deg ->  60mm
"""

from __future__ import annotations

import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from std_msgs.msg import Empty


HELP = """
================= 그리퍼 수동 조작 =================
  a / d      집게 회전 (servo_4)      -5도 / +5도
  A / D      집게 회전 빠르게        -20도 / +20도
  w / s      손가락 열기 / 닫기        -5도 / +5도
  W / S      손가락 열기/닫기 빠르게  -20도 / +20도

  o          활짝 열기   (-60도, 틈 211mm)
  g          트레이 파지 (-30도, 틈 138mm)
  r          회전 0도 복귀
  z          짧은변 자동정렬 다시 적용

  Enter      확실히 잡았음 -> 다음 단계(상승)로
  ESC        못 잡았음/떨어뜨림 -> 이 구간만 재시작
             (집게 열고 핸드오버 고도로 올라갔다가 REP 재탐지 후 다시 하강.
              상승 중에 눌러도 받는다 - 안 잡힌 걸 뒤늦게 알아챈 경우)
  q          종료(조작 포기)
====================================================
"""


class GripperTeleop(Node):
    def __init__(self) -> None:
        super().__init__("gripper_teleop")
        self.cmd_pub = self.create_publisher(Vector3, "/krac/gripper/cmd", 10)
        self.confirm_pub = self.create_publisher(Empty, "/krac/manual_grasp/confirm", 10)
        self.retry_pub = self.create_publisher(Empty, "/krac/manual_grasp/retry", 10)
        self.get_logger().info(
            "gripper_teleop up: /krac/gripper/cmd , /krac/manual_grasp/{confirm,retry}"
        )

    # x: rotation delta(deg), y: finger delta(deg), z: preset code
    #   z=0 none, 1 open wide, 2 grip tray, 3 rotation zero, 4 re-align to short side
    def _send(self, rot_d: float = 0.0, fing_d: float = 0.0, preset: float = 0.0) -> None:
        v = Vector3()
        v.x = float(rot_d)
        v.y = float(fing_d)
        v.z = float(preset)
        self.cmd_pub.publish(v)


def main() -> None:
    rclpy.init()
    node = GripperTeleop()
    print(HELP)

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while rclpy.ok():
            ch = sys.stdin.read(1)
            if ch == "q":
                print("종료.")
                break
            elif ch in ("\r", "\n"):
                node.confirm_pub.publish(Empty())
                print(">>> 파지 확정 신호 전송. 기체가 상승합니다.")
            elif ch == "\x1b":  # ESC
                node.retry_pub.publish(Empty())
                print(">>> 재시도 신호 전송. 집게 열고 올라갔다가 REP 재하강합니다.")
            elif ch == "a":
                node._send(rot_d=-5.0); print("회전 -5")
            elif ch == "d":
                node._send(rot_d=+5.0); print("회전 +5")
            elif ch == "A":
                node._send(rot_d=-20.0); print("회전 -20")
            elif ch == "D":
                node._send(rot_d=+20.0); print("회전 +20")
            elif ch == "w":
                node._send(fing_d=-5.0); print("손가락 열기 -5")
            elif ch == "s":
                node._send(fing_d=+5.0); print("손가락 닫기 +5")
            elif ch == "W":
                node._send(fing_d=-20.0); print("손가락 열기 -20")
            elif ch == "S":
                node._send(fing_d=+20.0); print("손가락 닫기 +20")
            elif ch == "o":
                node._send(preset=1.0); print("활짝 열기 (-60도)")
            elif ch == "g":
                node._send(preset=2.0); print("트레이 파지 (-30도)")
            elif ch == "r":
                node._send(preset=3.0); print("회전 0도 복귀")
            elif ch == "z":
                node._send(preset=4.0); print("짧은변 자동정렬 재적용")
            elif ch == "?":
                print(HELP)
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
