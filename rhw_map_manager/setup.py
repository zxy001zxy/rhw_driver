from setuptools import find_packages, setup

package_name = 'rhw_map_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', ['launch/map_manager.launch.py']),
        ('share/' + package_name + '/config', ['config/map_manager.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xwqf',
    maintainer_email='xwqf@todo.todo',
    description='Map and mode control services for APP integration.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_manager_node = rhw_map_manager.map_manager_node:main',
        ],
    },
)
