"""Launch file for rhw_task_scheduler — waypoint_manager + mission_bt_node."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    config_file = (
        Path(get_package_share_directory('rhw_task_scheduler'))
        / 'config'
        / 'task_scheduler.yaml'
    )

    # 是否启动 Web 可视化
    bt_viewer_arg = DeclareLaunchArgument(
        'bt_viewer', default_value='false',
        description='Launch BT Web Viewer (http://localhost:8765)',
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

    bt_web_viewer = Node(
        package='rhw_task_scheduler',
        executable='bt_web_viewer',
        name='bt_web_viewer',
        output='screen',
        condition=IfCondition(LaunchConfiguration('bt_viewer')),
    )

    return LaunchDescription([
        bt_viewer_arg,
        waypoint_manager,
        mission_bt,
        bt_web_viewer,
    ])
