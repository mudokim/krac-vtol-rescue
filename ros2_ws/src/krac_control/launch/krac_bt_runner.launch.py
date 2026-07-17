from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('krac_control')
    # bt_real: 실기체 canonical BT/params 를 기본값으로. (시뮬용 robust.xml 은 제거됨)
    default_xml = PathJoinSubstitution([pkg, 'bt', 'bt_rescue_test.xml'])
    default_params = PathJoinSubstitution([pkg, 'config', 'krac_bt_params_krac24.yaml'])
    default_mission_dir = PathJoinSubstitution([pkg, 'mission'])

    return LaunchDescription([
        DeclareLaunchArgument('bt_xml_path', default_value=default_xml),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('tick_rate_hz', default_value='10.0'),
        DeclareLaunchArgument('max_ticks', default_value='0'),
        DeclareLaunchArgument('print_bt_transitions', default_value='false'),
        DeclareLaunchArgument('mission_dir', default_value=default_mission_dir),
        DeclareLaunchArgument('sim_bypass', default_value='true'),
        DeclareLaunchArgument('mission_upload_stub_success', default_value='false'),
        DeclareLaunchArgument('gripper_stub_success', default_value='false'),
        DeclareLaunchArgument('enable_bt_viewer', default_value='false'),
        DeclareLaunchArgument('bt_viewer_direction', default_value='Vertical'),
        Node(
            package='krac_control',
            executable='krac_bt_runner',
            name='krac_bt_runner',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'bt_xml_path': LaunchConfiguration('bt_xml_path'),
                    'tick_rate_hz': LaunchConfiguration('tick_rate_hz'),
                    'max_ticks': LaunchConfiguration('max_ticks'),
                    'print_bt_transitions': LaunchConfiguration('print_bt_transitions'),
                    'mission_dir': LaunchConfiguration('mission_dir'),
                    'sim_bypass': LaunchConfiguration('sim_bypass'),
                    'mission_upload_stub_success': LaunchConfiguration('mission_upload_stub_success'),
                    'gripper_stub_success': LaunchConfiguration('gripper_stub_success'),
                    'enable_bt_viewer': LaunchConfiguration('enable_bt_viewer'),
                    'bt_viewer_direction': LaunchConfiguration('bt_viewer_direction'),
                }
            ],
        )
    ])
