import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 프리셋 각도 등 파라미터 파일
    config = os.path.join(
        get_package_share_directory('krac_gimbal'),
        'config', 'gimbal_presets.yaml')

    return LaunchDescription([
        Node(
            package='krac_gimbal',
            executable='gimbal_node',
            name='gimbal_node',
            output='screen',
            parameters=[config],
        ),
    ])
