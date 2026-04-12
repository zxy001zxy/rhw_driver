from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description() -> LaunchDescription:
    config_file = Path(get_package_share_directory('rhw_map_manager')) / 'config' / 'map_manager.yaml'

    return LaunchDescription([
        Node(
            package='rhw_map_manager',
            executable='map_manager_node',
            name='map_manager_node',
            output='screen',
            parameters=[str(config_file)],
        )
    ])
