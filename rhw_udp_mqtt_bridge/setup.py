from setuptools import find_packages, setup

package_name = 'rhw_udp_mqtt_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/udp_bridge.launch.py']),
        ('share/' + package_name + '/config', ['config/udp_mqtt_bridge.yaml']),
    ],
    install_requires=['setuptools', 'paho-mqtt', 'requests', 'cryptography'],
    zip_safe=True,
    maintainer='xwqf',
    maintainer_email='xwqf@todo.todo',
    description='ROS 2 package skeleton for UDP parsing, ROS topic publishing and MQTT forwarding.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'udp_bridge_node = rhw_udp_mqtt_bridge.udp_bridge_node:main',
            'mqtt_forwarder_node = rhw_udp_mqtt_bridge.mqtt_forwarder_node:main',
            'inspection_reporter_node = rhw_udp_mqtt_bridge.inspection_reporter_node:main',
        ],
    },
)
