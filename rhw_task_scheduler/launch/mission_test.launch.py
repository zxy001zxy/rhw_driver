"""Launch the mission flow test stack with selectable mock/real services."""
from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value: str) -> bool:
    return value.strip().lower() in ('1', 'true', 'yes', 'on')


def _service_name(real_name: str, test_name: str, use_real: bool) -> str:
    return real_name if use_real else test_name


def launch_setup(context, *args, **kwargs):
    config_file = (
        Path(get_package_share_directory('rhw_task_scheduler'))
        / 'config'
        / 'task_scheduler.yaml'
    )

    use_real_waypoints = _as_bool(LaunchConfiguration('use_real_waypoints').perform(context))
    use_real_navigation = _as_bool(LaunchConfiguration('use_real_navigation').perform(context))
    use_real_ptz = _as_bool(LaunchConfiguration('use_real_ptz').perform(context))
    use_real_recharge = _as_bool(LaunchConfiguration('use_real_recharge').perform(context))
    use_real_battery = _as_bool(LaunchConfiguration('use_real_battery').perform(context))
    launch_waypoint_manager = _as_bool(
        LaunchConfiguration('launch_waypoint_manager').perform(context)
    )
    bt_viewer = _as_bool(LaunchConfiguration('bt_viewer').perform(context))

    nodes = []

    if use_real_waypoints and launch_waypoint_manager:
        nodes.append(
            Node(
                package='rhw_task_scheduler',
                executable='waypoint_manager',
                name='waypoint_manager',
                output='screen',
                parameters=[str(config_file)],
            )
        )

    mock_node = Node(
        package='rhw_task_scheduler',
        executable='mission_test_mocks',
        name='mission_test_mocks',
        output='screen',
        parameters=[
            {
                'use_real_waypoints': use_real_waypoints,
                'use_real_navigation': use_real_navigation,
                'use_real_ptz': use_real_ptz,
                'use_real_recharge': use_real_recharge,
                'use_real_battery': use_real_battery,
                'map_name': LaunchConfiguration('map_name').perform(context),
                'waypoints_json': LaunchConfiguration('waypoints_json').perform(context),
            }
        ],
    )

    mission_params = [
        str(config_file),
        {
            'get_waypoints_service': _service_name(
                '/waypoint_manager/get_waypoints',
                '/test/waypoint_manager/get_waypoints',
                use_real_waypoints,
            ),
            'goal_service': _service_name(
                '/move_base_simple/goal',
                '/test/move_base_simple/goal',
                use_real_navigation,
            ),
            'nav_status_topic': _service_name(
                '/navigation_status',
                '/test/navigation_status',
                use_real_navigation,
            ),
            'ptz_absolute_move_service': _service_name(
                '/ptz/absolute_move',
                '/test/ptz/absolute_move',
                use_real_ptz,
            ),
            'ptz_capture_service': _service_name(
                '/ptz/capture_image',
                '/test/ptz/capture_image',
                use_real_ptz,
            ),
            'ptz_status_topic': _service_name(
                '/ptz/status',
                '/test/ptz/status',
                use_real_ptz,
            ),
            'recharge_service': _service_name(
                '/recharge',
                '/test/recharge',
                use_real_recharge,
            ),
            'battery_topic': _service_name(
                '/robot/battery_status',
                '/test/robot/battery_status',
                use_real_battery,
            ),
            'debug_print_tree_on_build': True,
            'debug_print_tree_on_tick': False,
        },
    ]

    mission_bt = Node(
        package='rhw_task_scheduler',
        executable='mission_bt_node',
        name='mission_bt_node',
        output='screen',
        parameters=mission_params,
    )

    nodes.extend([mock_node, mission_bt])

    if bt_viewer:
        nodes.append(
            Node(
                package='rhw_task_scheduler',
                executable='bt_web_viewer',
                name='bt_web_viewer',
                output='screen',
            )
        )

    return nodes


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_real_waypoints',
                default_value='false',
                description='Use the real waypoint_manager service',
            ),
            DeclareLaunchArgument(
                'use_real_navigation',
                default_value='false',
                description='Use the real navigation stack',
            ),
            DeclareLaunchArgument(
                'use_real_ptz',
                default_value='true',
                description='Use the real PTZ controller',
            ),
            DeclareLaunchArgument(
                'use_real_recharge',
                default_value='false',
                description='Use the real recharge service',
            ),
            DeclareLaunchArgument(
                'use_real_battery',
                default_value='false',
                description='Use the real battery topic',
            ),
            DeclareLaunchArgument(
                'launch_waypoint_manager',
                default_value='true',
                description='Launch waypoint_manager when using real waypoints',
            ),
            DeclareLaunchArgument(
                'map_name',
                default_value='factory_map',
                description='Mock map name',
            ),
            DeclareLaunchArgument(
                'waypoints_json',
                default_value='',
                description='Optional JSON overrides for mocked waypoints',
            ),
            DeclareLaunchArgument(
                'bt_viewer',
                default_value='false',
                description='Launch BT web viewer',
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
