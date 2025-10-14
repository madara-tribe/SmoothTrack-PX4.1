from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    # Run the px3 Python node directly from the source path
    px3 = ExecuteProcess(
        cmd=['/usr/bin/env', 'python3', '/ros2_ws/work/src/hw_px3/hw_px3/main.py'],
        output='screen'
    )

    px2 = Node(
        package='px2',
        executable='px2',
        output='screen',
        parameters=[{
            'device_path': '/dev/video2',
            'fov': 70.0,
            'kp': 1.0,
            'lost_max_frames': 15,
            'track_class': -1,
            'save_frames': False,
            'max_step_deg': 8.0,
            'center_on_start': True,
            'tracker_type': 'KCF',
            'enforce_bgr8': True
        }]
    )

    return LaunchDescription([px3, px2])
