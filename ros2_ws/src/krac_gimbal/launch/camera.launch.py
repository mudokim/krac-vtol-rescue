import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # RTSP/디코드 파라미터 파일
    config = os.path.join(
        get_package_share_directory('krac_gimbal'),
        'config', 'camera.yaml')

    return LaunchDescription([
        Node(
            package='krac_gimbal',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[config],
        ),
    ])
