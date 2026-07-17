import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """짐벌 각도 제어(gimbal_node) + 카메라 영상 브리지(camera_node) 동시 실행."""
    share = get_package_share_directory('krac_gimbal')
    gimbal_cfg = os.path.join(share, 'config', 'gimbal_presets.yaml')
    camera_cfg = os.path.join(share, 'config', 'camera.yaml')

    return LaunchDescription([
        Node(
            package='krac_gimbal',
            executable='gimbal_node',
            name='gimbal_node',
            output='screen',
            parameters=[gimbal_cfg],
        ),
        Node(
            package='krac_gimbal',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[camera_cfg],
        ),
    ])
