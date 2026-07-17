import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """실기체용 MAVROS 브링업 (bt_real).

    시뮬용 sitl_vtol.launch.py 에서 'make px4_sitl'(Gazebo/PX4 SITL) 를 제거한
    버전이다. /mavros/... 토픽·서비스 이름은 시뮬과 동일하므로 BT/제어 코드는
    수정 없이 그대로 동작한다. 유일한 실질 차이는 mavros_params_real.yaml 의
    fcu_url(실제 FC 연결) 이다.

    나머지 노드(카메라/짐벌/비전/BT runner/뷰어)는 scripts/run_real_bt.sh 가
    오케스트레이션한다. 이 런치는 MAVROS 만(+선택적 logger) 올린다.
    """
    mission_pkg = 'krac_mission'
    control_pkg = 'krac_control'

    mission_share = get_package_share_directory(mission_pkg)

    default_mavros_config = os.path.join(mission_share, 'config', 'mavros_params_real.yaml')
    default_waypoints_config = os.path.join(mission_share, 'config', 'waypoints.yaml')

    mavros_config = LaunchConfiguration('mavros_config')
    waypoints_config = LaunchConfiguration('waypoints_config')
    start_mavros = LaunchConfiguration('start_mavros')
    start_logger = LaunchConfiguration('start_logger')

    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        output='screen',
        parameters=[mavros_config],
        namespace='mavros',
        condition=IfCondition(start_mavros),
    )

    logger_node = Node(
        package='krac_utils',
        executable='competition_logger',
        name='competition_logger',
        output='screen',
        parameters=[waypoints_config],
        condition=IfCondition(start_logger),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'mavros_config',
            default_value=default_mavros_config,
            description='실기체 MAVROS 파라미터 YAML (fcu_url 등)',
        ),
        DeclareLaunchArgument(
            'waypoints_config',
            default_value=default_waypoints_config,
            description='KRAC waypoint/mission 파라미터 YAML',
        ),
        DeclareLaunchArgument('start_mavros', default_value='true'),
        # BT runner 가 competition_logger 를 대신 띄우지 않으므로, 대회 로깅이
        # 필요하면 true 로. 기본은 off.
        DeclareLaunchArgument('start_logger', default_value='false'),
        mavros_node,
        logger_node,
    ])
