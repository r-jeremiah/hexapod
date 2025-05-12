from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the hexapod control node
        Node(
            package='hexapod_control',
            executable='hexapod_control_node',
            name='hexapod_control_node',
            output='screen',
        ),

        # Launch the MPU6050 publisher node
        Node(
            package='mpu6050_publisher',
            executable='mpu6050_node',
            name='mpu6050_node',
            output='screen',
        ),

        # Launch the hexapod stabilizer node
        Node(
            package='hexapod_control',
            executable='hexapod_stabilizer_node',
            name='hexapod_stabilizer_node',
            output='screen',
        ),

        # Launch the camera node from hexapod_vision
        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera_node',
            parameters=[{'format': 'RGB888'}],
            output='screen',
        ),

        # Launch the web video server node
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
            output='screen',
        ),

        # Launch the AMG8833 publisher node from hexapod_vision
        Node(
            package='hexapod_vision',
            executable='amg8833_publisher',
            name='amg8833_publisher',
            output='screen',
        ),

        # Launch the websocket server node
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='websocket_server_node',
            output='screen',
        ),
    ])