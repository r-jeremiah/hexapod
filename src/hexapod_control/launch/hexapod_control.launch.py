from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Hexapod Control Node
        Node(
            package='hexapod_control',
            executable='hexapod_control_node',
            name='hexapod_control_node',
            output='screen',
            parameters=[]  # Add any parameters if needed
        ),
        # Hexapod Stabilizer Node
        Node(
            package='hexapod_control',
            executable='hexapod_stabilizer_node',
            name='hexapod_stabilizer_node',
            output='screen',
            parameters=[]  # Add any parameters if needed
        ),

        Node(
            package='websocket_server',
            executable='websocket_server_node',
            name='websocket_server_node',
            output='screen',
        )
    ])