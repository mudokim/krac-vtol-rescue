#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64  # Gazebo 토픽 제어용 메시지
import time

class GripperController(Node):
    def __init__(self):
        super().__init__('gripper_controller')
        
        # 기체 이름(standard_vtol_0)에 맞게 퍼블리셔 생성
        # servo_4: 그리퍼 Z축 회전
        # servo_5, servo_6: 왼쪽/오른쪽 그리퍼 손가락
        self.pub_s4 = self.create_publisher(Float64, '/model/amsr_vtol_0/servo_4', 10)
        self.pub_s5 = self.create_publisher(Float64, '/model/amsr_vtol_0/servo_5', 10)
        self.pub_s6 = self.create_publisher(Float64, '/model/amsr_vtol_0/servo_6', 10)
        
        self.get_logger().info('그리퍼 제어 노드 준비 완료! (Bridge가 켜져 있어야 합니다)')

    def control_gripper(self, action):
        msg = Float64()
        
        if action == "rotation_l":
            self.get_logger().info('>>> 왼쪽 회전 (Rotation Left)')
            msg.data = 1.57  # 약 90도 회전 (필요에 따라 각도 조절)
            self.pub_s4.publish(msg)
            
        elif action == "rotation_r":
            self.get_logger().info('>>> 오른쪽 회전 (Rotation Right)')
            msg.data = -1.57 # 약 -90도 회전
            self.pub_s4.publish(msg)
            
        elif action == "rotation_0":
            self.get_logger().info('>>> 정면 복귀 (Rotation Center)')
            msg.data = 0.0   # 0도 (원위치)
            self.pub_s4.publish(msg)
            
        elif action == "grab":
            self.get_logger().info('>>> 물체 파지 (Grab)')
            msg.data = 0.5 
            self.pub_s5.publish(msg)
            msg.data = 0.5
            self.pub_s6.publish(msg)
            
        elif action == "release":
            self.get_logger().info('>>> 물체 방출 (Release)')
            msg.data = -1.2
            self.pub_s5.publish(msg)
            msg.data = -1.2
            self.pub_s6.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GripperController()

    print("\n==================================")
    print(" 🕹️ 그리퍼 & 회전 제어 테스트")
    print(" [0] 왼쪽 회전 (Rotation Left)")
    print(" [1] 오른쪽 회전 (Rotation Right)")
    print(" [2] 물체 파지 (Grab)")
    print(" [3] 물체 방출 (Release)")
    print(" [4] 정면 복귀 (Rotation Center)")
    print(" [q] 종료")
    print("==================================\n")

    try:
        while rclpy.ok():
            cmd = input("명령을 입력하세요 (0/1/2/3/4/q): ")
            if cmd == '0':
                node.control_gripper("rotation_l")
            elif cmd == '1':
                node.control_gripper("rotation_r")
            elif cmd == '2':
                node.control_gripper("grab")
            elif cmd == '3':
                node.control_gripper("release")
            elif cmd == '4':
                node.control_gripper("rotation_0")
            elif cmd == 'q':
                break
            else:
                print("잘못된 입력입니다. 0, 1, 2, 3, 4, q 중에서 입력해주세요.")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
