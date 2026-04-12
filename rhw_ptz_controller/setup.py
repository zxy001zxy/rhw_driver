from setuptools import find_packages, setup

package_name = 'rhw_ptz_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ptz_controller.launch.py']),
        ('share/' + package_name + '/config', ['config/ptz_controller.yaml']),
    ],
    install_requires=['setuptools', 'requests'],
    zip_safe=True,
    maintainer='xwqf',
    maintainer_email='xwqf@todo.todo',
    description='ROS 2 node wrapping ISAPI PTZ camera control.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ptz_controller_node = rhw_ptz_controller.ptz_controller_node:main',
            'camera_publisher_node = rhw_ptz_controller.camera_publisher_node:main',
        ],
    },
)
