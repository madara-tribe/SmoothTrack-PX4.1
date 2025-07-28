from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='px2',
            executable='px2',
            name='px2_node',
            output='screen',
            parameters=[{'camera_path': '/dev/video2'}]
        ),
        Node(
            package='hw_px3',
            executable='hw_px3',
            name='hw_px3_node',
            output='screen',
            parameters=[{'arduino_port': '/dev/ttyACM0'}]
        )
    ])

