from setuptools import setup, find_packages

package_name = 'rhw_task_scheduler'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/task_scheduler.yaml']),
        ('share/' + package_name + '/launch', ['launch/task_scheduler.launch.py']),
    ],
    install_requires=['setuptools', 'py_trees', 'paho-mqtt'],
    zip_safe=True,
    maintainer='xwqf',
    maintainer_email='xwqf@todo.todo',
    description='行为树驱动的航点任务调度系统',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'waypoint_manager = rhw_task_scheduler.waypoint_manager:main',
            'mission_bt_node = rhw_task_scheduler.mission_bt_node:main',
            'mock_mission_runner = rhw_task_scheduler.mock_mission_runner:main',
            'mock_service_responder = rhw_task_scheduler.mock_service_responder:main',
            'bt_web_viewer = rhw_task_scheduler.bt_web_viewer:main',
        ],
    },
)
