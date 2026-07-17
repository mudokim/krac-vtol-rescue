import rclpy
from rclpy.node import Node
import os
from ament_index_python.packages import get_package_share_directory # ROS2 경로 찾기 필수템
from ultralytics import YOLO

class SurvivorYolo(Node):
    def __init__(self):
        super().__init__('survivor_yolo')
        
        # ---------------------------------------------------
        # [핵심] 설치된 패키지의 share 폴더 경로를 자동으로 찾습니다.
        # ---------------------------------------------------
        package_share_directory = get_package_share_directory('krac_vision')
        
        # weights 폴더와 best.pt 파일 경로 조합
        model_path = os.path.join(package_share_directory, 'weights', 'best.pt')
        
        self.get_logger().info(f'모델 로딩 중... 경로: {model_path}')
        
        # 모델 로드 (경로가 확실하므로 에러가 안 납니다)
        try:
            self.model = YOLO(model_path)
            self.get_logger().info('YOLO 모델 로딩 성공!')
        except Exception as e:
            self.get_logger().error(f'모델 로딩 실패: {e}')

        # ... (이후 Subscriber 등 코드 작성) ...

def main(args=None):
    rclpy.init(args=args)
    node = SurvivorYolo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
   
