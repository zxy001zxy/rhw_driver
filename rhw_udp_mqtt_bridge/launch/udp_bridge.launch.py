from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    config_file = Path(get_package_share_directory('rhw_udp_mqtt_bridge')) / 'config' / 'udp_mqtt_bridge.yaml'

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_mqtt_forwarder',
            default_value='false',
            description='Whether to launch mqtt_forwarder_node together with udp_bridge_node',
        ),
        Node(
            package='rhw_udp_mqtt_bridge',
            executable='udp_bridge_node',
            name='udp_bridge_node',
            output='screen',
            parameters=[str(config_file)],
        ),
        Node(
            package='rhw_udp_mqtt_bridge',
            executable='mqtt_forwarder_node',
            name='mqtt_forwarder_node',
            output='screen',
            parameters=[str(config_file)],
            condition=IfCondition(LaunchConfiguration('enable_mqtt_forwarder')),
        ),
    ])
