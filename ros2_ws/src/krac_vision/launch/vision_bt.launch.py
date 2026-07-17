import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('krac_vision')

    # Prefer an explicit model path from the current workspace.
    # This avoids accidentally loading a stale best.pt from ~/vtol-aam-rescue.
    env_model = os.environ.get('VISION_MODEL_PATH', '')
    share_model = Path(pkg_share) / 'weights' / 'best.pt'
    if env_model:
        default_model = env_model
    else:
        default_model = str(share_model)

    model = LaunchConfiguration('model')
    threshold = LaunchConfiguration('threshold')
    obb_confidence = LaunchConfiguration('obb_confidence')
    tracking_hold_sec = LaunchConfiguration('tracking_hold_sec')
    inference_interval_sec = LaunchConfiguration('inference_interval_sec')
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
            'obb_confidence',
            default_value='0.20',
            description='YOLO OBB confidence threshold for /vision/target_error tracker'
        ),
        DeclareLaunchArgument(
            'tracking_hold_sec',
            default_value='75.0',
            description='Seconds to keep TargetError valid using Kalman prediction after a brief visual dropout'
        ),
        DeclareLaunchArgument(
            'inference_interval_sec',
            default_value='0.25',
            description='Minimum seconds between YOLO OBB inference calls while a tracked target is held'
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
            description='Shared camera image topic for yolo_node and vision_tracker '
                        '(must match scripts/run_gz_image_republisher.sh ros_topic)'
        ),
        DeclareLaunchArgument(
            'start_yolo_detector',
            default_value='true',
            description='Start the point-based yolo_node (/vision/detections). '
                        'Only needed for DropOperation BT nodes (ActivateYOLO/'
                        'IsObjectDetected/ExecuteSearchPattern); the rescue-leg '
                        'precision landing path only needs vision_tracker below, '
                        'so mission profiles without a drop zone (e.g. krac24_split) '
                        'can set this to false to save a full extra CPU YOLO pass per frame.'
        ),

        # Point-based detector: feeds /vision/detections, used by DropOperation's
        # ActivateYOLO / IsObjectDetected / ExecuteSearchPattern BT nodes.
        Node(
            condition=IfCondition(LaunchConfiguration('start_yolo_detector')),
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
        ),

        # TargetError-based tracker: feeds /vision/target_error, consumed by
        # precision_lander (RescueOperation / FinalLanding BT nodes).
        Node(
            package='krac_vision',
            executable='vision_tracker',
            name='vision_tracker_node',
            output='screen',
            parameters=[{
                'model': model,
                'image_topic': image_topic,
                'obb_confidence': obb_confidence,
                'tracking_hold_sec': tracking_hold_sec,
                'inference_interval_sec': inference_interval_sec,
            }],
        ),

        # BT precision phases toggle this node via /precision_lander/enable.
        Node(
            package='krac_control',
            executable='precision_lander',
            name='precision_lander',
            output='screen',
        ),
    ])
