from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='px2',
            executable='px2',
            name='px2_node',
            output='screen'
        ),
        Node(
            package='hw_px3',
            executable='main',
            name='hw_px3_node',
            output='screen'
        )
    ])

