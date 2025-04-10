from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera_node',
            parameters=[{'format': 'RGB888'}]
        ),
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
        ),
        Node(
            package='hexapod_vision',
            executable='amg8833_publisher',
            name='amg8833_publisher',
        )
    ])
