from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from rcl_interfaces.msg import ParameterType

def generate_launch_description():
    # ----- Common QoL -----
    color = SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1')

    # ----- Declare args (override at CLI) -----
    # package/executable names (change if your install uses different ones)
    arg_px2_pkg  = DeclareLaunchArgument('px2_pkg',  default_value='px2',  description='px2 package')
    arg_px2_exec = DeclareLaunchArgument('px2_exec', default_value='px2',  description='px2 executable')
    arg_px3_pkg  = DeclareLaunchArgument('px3_pkg',  default_value='px3',  description='px3 package')
    arg_px3_exec = DeclareLaunchArgument('px3_exec', default_value='px3',  description='px3 executable')

    # camera + tracking (px2)
    arg_camera   = DeclareLaunchArgument('camera_path',   default_value='/dev/video2', description='V4L2 camera path for px2')
    arg_hfov     = DeclareLaunchArgument('hfov_deg',      default_value='62.0',        description='Camera horizontal FOV (deg)')
    arg_vfov     = DeclareLaunchArgument('vfov_deg',      default_value='48.0',        description='Camera vertical FOV (deg)')
    arg_kp       = DeclareLaunchArgument('kp',            default_value='1.0',         description='Degrees of servo per degree of image error')
    arg_invert   = DeclareLaunchArgument('invert_servo',  default_value='false',       description='Invert servo direction')
    arg_lost     = DeclareLaunchArgument('lost_max_frames', default_value='15',        description='Frames allowed lost before re-detect')
    arg_cls      = DeclareLaunchArgument('track_class',   default_value='-1',          description='Target class id (-1:any)')
    arg_save     = DeclareLaunchArgument('save_frames',    default_value='false',      description='Save debug frames')

    # serial (px3)
    arg_serial   = DeclareLaunchArgument('serial_port',   default_value='/dev/ttyACM0', description='Arduino serial device')
    arg_baud     = DeclareLaunchArgument('baud',          default_value='9600',         description='Arduino baud')
    arg_gap      = DeclareLaunchArgument('min_interval_s',default_value='0.10',         description='Min seconds between servo writes')

    # ----- Nodes -----
    px3_node = Node(
        package=LaunchConfiguration('px3_pkg'),
        executable=LaunchConfiguration('px3_exec'),
        name='px3',
        output='screen',
        emulate_tty=True,
        parameters=[{
            # px3 expects AbsResult on "inference" and writes to serial. :contentReference[oaicite:1]{index=1}
            'serial_port': LaunchConfiguration('serial_port'),
            'baud': ParameterValue(LaunchConfiguration('baud'), value_type=ParameterType.PARAMETER_INTEGER),
            'min_interval_s': ParameterValue(LaunchConfiguration('min_interval_s'), value_type=ParameterType.PARAMETER_DOUBLE),
        }]
    )

    # Optional: give px3 ~1s head start (not strictly needed since we use px3_ready)
    px2_node = TimerAction(
        period=1.0,
        actions=[Node(
            package=LaunchConfiguration('px2_pkg'),
            executable=LaunchConfiguration('px2_exec'),
            name='px2',
            output='screen',
            emulate_tty=True,
            parameters=[{
                # px2 publishes AbsResult on "inference" after detect/track loop. :contentReference[oaicite:2]{index=2}
                'device_path': LaunchConfiguration('camera_path'),
                'hfov_deg':   ParameterValue(LaunchConfiguration('hfov_deg'), value_type=ParameterType.PARAMETER_DOUBLE),
                'vfov_deg':   ParameterValue(LaunchConfiguration('vfov_deg'), value_type=ParameterType.PARAMETER_DOUBLE),
                'kp':         ParameterValue(LaunchConfiguration('kp'),       value_type=ParameterType.PARAMETER_DOUBLE),
                'invert_servo': ParameterValue(LaunchConfiguration('invert_servo'), value_type=ParameterType.PARAMETER_BOOL),
                'lost_max_frames': ParameterValue(LaunchConfiguration('lost_max_frames'), value_type=ParameterType.PARAMETER_INTEGER),
                'track_class': ParameterValue(LaunchConfiguration('track_class'), value_type=ParameterType.PARAMETER_INTEGER),
                'save_frames': ParameterValue(LaunchConfiguration('save_frames'), value_type=ParameterType.PARAMETER_BOOL),
            }]
        )]
    )

    return LaunchDescription([
        color,
        arg_px2_pkg, arg_px2_exec, arg_px3_pkg, arg_px3_exec,
        arg_camera, arg_hfov, arg_vfov, arg_kp, arg_invert, arg_lost, arg_cls, arg_save,
        arg_serial, arg_baud, arg_gap,
        px3_node,
        px2_node
    ])


