#!/usr/bin/env python3
"""실제 드론용 SIYI A8 mini 카메라 브링업 (bt_real).

시뮬레이션의 gz_image_republisher / ros_gz_image image_bridge 를 대체한다.
SIYI A8 mini 의 RTSP 스트림을 받아 비전 스택(yolo_node / vision_tracker)이
구독하는 공유 이미지 토픽으로 발행한다.

기본 image_topic 은 '/image_raw' 로, krac_vision 의 vision_bt.launch.py 기본값
(image_topic:=/image_raw)과 정확히 일치시킨다. 따라서 시뮬에서 쓰던 비전 런치를
그대로 재사용할 수 있다.

sensor_msgs/Image (bgr8) + sensor_msgs/CameraInfo 를 SensorDataQoS(best-effort)로
발행한다. 비전 노드들도 qos_profile_sensor_data 로 구독하므로 QoS 가 맞는다.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rtsp_url = LaunchConfiguration('rtsp_url')
    image_topic = LaunchConfiguration('image_topic')
    frame_id = LaunchConfiguration('frame_id')
    rtsp_transport = LaunchConfiguration('rtsp_transport')
    fps_limit = LaunchConfiguration('fps_limit')
    publish_compressed = LaunchConfiguration('publish_compressed')
    camera_info_url = LaunchConfiguration('camera_info_url')

    return LaunchDescription([
        DeclareLaunchArgument(
            'rtsp_url',
            default_value='rtsp://192.168.144.25:8554/main.264',
            description='SIYI A8 mini RTSP 스트림 주소 (main=1080p, sub=저해상도). '
                        'A8 mini 기본 IP=192.168.144.25.',
        ),
        DeclareLaunchArgument(
            'image_topic',
            # 실기체 네이티브 규약(/camera/image_raw)으로 통일한다.
            # my_package 카메라 노드의 검증된 기본값이자, krac_gimbal/camera.yaml,
            # landing_marker.py(하드코딩)와도 일치하는 토픽이다.
            # 비전 스택(yolo_node/vision_tracker)은 image_topic:=/camera/image_raw 로
            # 이 토픽을 구독하도록 실행한다(run_real_bt.sh 에서 ROS_IMAGE_TOPIC 로 주입).
            default_value='/camera/image_raw',
            description='발행할 이미지 토픽. krac_vision vision_bt.launch.py 의 '
                        'image_topic 인자와 동일하게 유지할 것(/camera/image_raw).',
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='siyi_a8_camera',
            description='이미지/카메라인포 header.frame_id',
        ),
        DeclareLaunchArgument(
            'rtsp_transport',
            default_value='tcp',
            description='RTSP 전송 방식: tcp(안정) | udp(저지연)',
        ),
        DeclareLaunchArgument(
            'fps_limit',
            # 0.0 = 무제한. 비전은 내부에서 max_rate_hz / inference_interval 로
            # 다시 스로틀하므로 카메라단은 무제한으로 두고 최신 프레임을 흘려도 된다.
            default_value='0.0',
            description='발행 FPS 상한 (0.0=무제한)',
        ),
        DeclareLaunchArgument(
            'publish_compressed',
            default_value='false',
            description='<image_topic>/compressed 로 JPEG 도 발행할지 여부',
        ),
        DeclareLaunchArgument(
            'camera_info_url',
            default_value='',
            description='camera_info YAML (file:// 또는 package://). 비우면 해상도만 채운 '
                        '최소 CameraInfo 발행.',
        ),

        Node(
            package='my_package',
            executable='siyi_camera',
            name='siyi_camera_node',
            output='screen',
            parameters=[{
                'rtsp_url': rtsp_url,
                'image_topic': image_topic,
                'frame_id': frame_id,
                'rtsp_transport': rtsp_transport,
                'fps_limit': fps_limit,
                'publish_compressed': publish_compressed,
                'camera_info_url': camera_info_url,
                'reconnect_period_sec': 3.0,
            }],
        ),
    ])
