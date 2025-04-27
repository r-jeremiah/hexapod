from setuptools import setup

package_name = 'hexapod_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/vision.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hexapod',
    maintainer_email='hexapod@todo.todo',
    description='Hexapod vision package with camera and AMG8833',
    license='MIT',
    entry_points={
        'console_scripts': [
            'amg8833_publisher = hexapod_vision.amg8833_publisher:main',
        ],
    },


)
