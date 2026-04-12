from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    config_file = Path(get_package_share_directory('rhw_udp_mqtt_bridge')) / 'config' / 'udp_mqtt_bridge.yaml'

    return LaunchDescription([
        Node(
            package='rhw_udp_mqtt_bridge',
            executable='udp_bridge_node',
            name='udp_bridge_node',
            output='screen',
            parameters=[str(config_file)],
        ),
    ])
