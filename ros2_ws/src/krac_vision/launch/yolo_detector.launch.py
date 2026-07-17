import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('krac_vision')

    # Prefer the source-tree weight file during development.
    repo_model = Path.home() / 'vtol-aam-rescue' / 'ros2_ws' / 'src' / 'krac_vision' / 'weights' / 'best.pt'
    share_model = Path(pkg_share) / 'weights' / 'best.pt'

    if repo_model.exists():
        default_model = str(repo_model)
    else:
        default_model = str(share_model)

    model = LaunchConfiguration('model')
    threshold = LaunchConfiguration('threshold')
    device = LaunchConfiguration('device')
    initial_target = LaunchConfiguration('initial_target')
    image_topic = LaunchConfiguration('image_topic')

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value=default_model,
            description='Path to YOLO model weights, e.g. best.pt'
        ),

        DeclareLaunchArgument(
            'threshold',
            default_value='0.5',
            description='YOLO confidence threshold'
        ),

        DeclareLaunchArgument(
            'device',
            default_value='cpu',
            description='Inference device: cpu or cuda:0'
        ),

        # Do not use "off" here.
        # In ROS/YAML-style parameter parsing, "off" can become bool False.
        DeclareLaunchArgument(
            'initial_target',
            default_value='disabled',
            description='Initial target class: disabled, all, basket, drop_zone, vertiport'
        ),

        DeclareLaunchArgument(
            'image_topic',
            default_value='/image_raw',
            description='ROS image topic to subscribe'
        ),

        Node(
            package='krac_vision',
            executable='yolo_node',
            name='yolo_detector_node',
            output='screen',
            parameters=[{
                'model': model,
                'threshold': threshold,
                'device': device,
                'initial_target': initial_target,
                'image_topic': image_topic,
            }],
        )
    ])
