# filepath: /home/hexapod/ros2_ws/src/hexapod_control/setup.py
from setuptools import setup

package_name = 'hexapod_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/hexapod_control.launch.py']),
        ('share/' + package_name + '/launch', ['launch/arise_botogs.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hexapod',
    maintainer_email='hexapod@todo.todo',
    description='Hexapod control node for leg movement',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'hexapod_control_node = hexapod_control.hexapod_control_node:main',
            'hexapod_stabilizer_node = hexapod_control.hexapod_stabilizer_node:main',
        ],
    },
)