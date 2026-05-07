from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description() -> LaunchDescription:
    config_file = Path(get_package_share_directory('rhw_ptz_controller')) / 'config' / 'ptz_controller.yaml'

    return LaunchDescription([
        SetEnvironmentVariable('PYTHONNOUSERSITE', '1'),
        Node(
            package='rhw_ptz_controller',
            executable='ptz_controller_node',
            name='ptz_controller_node',
            output='screen',
            parameters=[str(config_file)],
        ),
        Node(
            package='rhw_ptz_controller',
            executable='camera_publisher_node',
            name='camera_publisher_node',
            output='screen',
            parameters=[str(config_file)],
        ),
        Node(
            package='rhw_ptz_controller',
            executable='camera_publisher_node',
            name='thermal_camera_publisher_node',
            output='screen',
            parameters=[str(config_file)],
        ),
    ])
