import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped, Point
from cv_bridge import CvBridge
from ultralytics import YOLO
import os
from ament_index_python.packages import get_package_share_directory

class LandingControlNode(Node):
    def __init__(self):
        super().__init__('landing_control_node')
        
        # 1. 모델 로드 (학습시킨 best.pt 경로)
        pkg_share = get_package_share_directory('krac_vision')
        model_path = os.path.join(pkg_share, 'weights', 'best.pt')
        self.model = YOLO(model_path)
        
        self.bridge = CvBridge()

        # 2. 토픽 설정
        # 카메라 영상 구독
        self.sub_img = self.create_subscription(Image, '/camera/image_raw', self.img_callback, 10)
        
        # [핵심] 드론에게 속도 명령을 보내는 토픽 (MAVROS 사용 시)
        self.pub_vel = self.create_publisher(TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)

        # 3. 제어 파라미터 (P-Gain)
        # 이 값을 조절해서 반응 속도를 튜닝합니다.
        self.kp_x = 0.005  # 가로 오차 보정 계수
        self.kp_y = 0.005  # 세로 오차 보정 계수
        self.kp_z = 0.5    # 하강 속도 계수
        self.target_class_id = 1 # 버티포트 클래스 번호

    def img_callback(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        height, width, _ = cv_img.shape
        img_center_x, img_center_y = width // 2, height // 2

        # YOLO 추론
        results = self.model(cv_img, verbose=False)
        
        vel_cmd = TwistStamped()
        detected = False

        for result in results:
            for box in result.boxes:
                if int(box.cls[0]) == self.target_class_id:
                    # 중심점 계산
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                    
                    # ------------------------------------------------
                    # [핵심 로직] 공유해주신 깃허브의 알고리즘 적용 부분
                    # ------------------------------------------------
                    
                    # 1. 픽셀 오차 계산 (화면 중심 - 물체 중심)
                    # 카메라 좌표계에 따라 부호가 다를 수 있으니 실제 테스트 필수!
                    err_x = img_center_x - cx
                    err_y = img_center_y - cy 
                    
                    # 2. 속도 명령 생성 (P-Control)
                    # 드론 좌표계: x는 전진, y는 좌측 이동이라고 가정
                    # (좌표계 변환이 필요할 수 있습니다. 아래는 예시)
                    vel_cmd.twist.linear.y = float(err_x * self.kp_x) 
                    vel_cmd.twist.linear.x = float(err_y * self.kp_y)
                    
                    # 3. 고도 제어 (물체가 보이면 천천히 하강)
                    # 오차가 작을 때(정렬되었을 때)만 하강하려면 조건문 추가
                    if abs(err_x) < 50 and abs(err_y) < 50:
                        vel_cmd.twist.linear.z = -0.3 # 0.3m/s로 하강
                    else:
                        vel_cmd.twist.linear.z = 0.0 # 정렬 전엔 고도 유지

                    detected = True
                    break

        if detected:
            self.pub_vel.publish(vel_cmd)
            self.get_logger().info(f"접근 중! V_x:{vel_cmd.twist.linear.x:.2f}, V_y:{vel_cmd.twist.linear.y:.2f}")
        else:
            # 물체를 놓치면 정지 (또는 상승)
            vel_cmd.twist.linear.x = 0.0
            vel_cmd.twist.linear.y = 0.0
            vel_cmd.twist.linear.z = 0.0
            self.pub_vel.publish(vel_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = LandingControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
