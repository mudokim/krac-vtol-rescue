import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """짐벌-큐 접근 코디네이터(approach_node).
    gimbal_node, 카메라, krac_vision 트래커가 함께 떠 있어야 동작한다."""
    config = os.path.join(
        get_package_share_directory('krac_gimbal'),
        'config', 'approach.yaml')

    return LaunchDescription([
        Node(
            package='krac_gimbal',
            executable='approach_node',
            name='approach_node',
            output='screen',
            parameters=[config],
        ),
    ])
