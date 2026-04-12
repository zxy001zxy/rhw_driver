"""Launch file for rhw_task_scheduler — waypoint_manager + mission_bt_node."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    config_file = (
        Path(get_package_share_directory('rhw_task_scheduler'))
        / 'config'
        / 'task_scheduler.yaml'
    )

    waypoint_manager = Node(
        package='rhw_task_scheduler',
        executable='waypoint_manager',
        name='waypoint_manager',
        output='screen',
        parameters=[str(config_file)],
    )

    mission_bt = Node(
        package='rhw_task_scheduler',
        executable='mission_bt_node',
        name='mission_bt_node',
        output='screen',
        parameters=[str(config_file)],
    )

    return LaunchDescription([waypoint_manager, mission_bt])
