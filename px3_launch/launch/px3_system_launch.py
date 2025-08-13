from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from rcl_interfaces.msg import ParameterType

def generate_launch_description():
    color = SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1')

    # Allow overriding package/executable names at CLI
    arg_px2_pkg  = DeclareLaunchArgument('px2_pkg',  default_value='px2',      description='px2 package')
    arg_px2_exec = DeclareLaunchArgument('px2_exec', default_value='px2',      description='px2 executable')
    arg_px3_pkg  = DeclareLaunchArgument('px3_pkg',  default_value='hw_px3',   description='px3 package')
    arg_px3_exec = DeclareLaunchArgument('px3_exec', default_value='hw_px3',   description='px3 executable')

    # Camera + tracking (px2)
    arg_camera   = DeclareLaunchArgument('camera_path',     default_value='/dev/video2')
    arg_hfov     = DeclareLaunchArgument('hfov_deg',        default_value='62.0')
    arg_vfov     = DeclareLaunchArgument('vfov_deg',        default_value='48.0')
    arg_ctrlmode = DeclareLaunchArgument('control_mode',    default_value='abs')  # abs | inc
    arg_kp       = DeclareLaunchArgument('kp',              default_value='1.0')
    arg_kpx      = DeclareLaunchArgument('kpx_deg_per_px',  default_value='1.0')  # used in inc
    arg_invert   = DeclareLaunchArgument('invert_servo',    default_value='false')
    arg_center   = DeclareLaunchArgument('center_on_start', default_value='true')
    arg_lost     = DeclareLaunchArgument('lost_max_frames', default_value='15')
    arg_cls      = DeclareLaunchArgument('track_class',     default_value='-1')
    arg_save     = DeclareLaunchArgument('save_frames',     default_value='false')

    # Serial (px3)
    arg_serial   = DeclareLaunchArgument('serial_port',     default_value='/dev/ttyACM0')
    arg_baud     = DeclareLaunchArgument('baud',            default_value='9600')
    arg_gap      = DeclareLaunchArgument('min_interval_s',  default_value='0.10')

    # px3 first: opens serial, writes 90°, publishes px3_ready (latched). :contentReference[oaicite:1]{index=1}
    px3_node = Node(
        package=LaunchConfiguration('px3_pkg'),
        executable=LaunchConfiguration('px3_exec'),
        name='px3',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud': ParameterValue(LaunchConfiguration('baud'), value_type=ParameterType.PARAMETER_INTEGER),
            'min_interval_s': ParameterValue(LaunchConfiguration('min_interval_s'), value_type=ParameterType.PARAMETER_DOUBLE),
        }]
    )

    # px2 after a short delay: waits for px3_ready, then DETECT→TRACK. 
    px2_node = TimerAction(
        period=1.0,
        actions=[Node(
            package=LaunchConfiguration('px2_pkg'),
            executable=LaunchConfiguration('px2_exec'),
            name='px2',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'device_path': LaunchConfiguration('camera_path'),
                'hfov_deg':   ParameterValue(LaunchConfiguration('hfov_deg'), value_type=ParameterType.PARAMETER_DOUBLE),
                'vfov_deg':   ParameterValue(LaunchConfiguration('vfov_deg'), value_type=ParameterType.PARAMETER_DOUBLE),
                'control_mode': LaunchConfiguration('control_mode'),
                'kp':         ParameterValue(LaunchConfiguration('kp'),       value_type=ParameterType.PARAMETER_DOUBLE),
                'kpx_deg_per_px': ParameterValue(LaunchConfiguration('kpx_deg_per_px'), value_type=ParameterType.PARAMETER_DOUBLE),
                'invert_servo':   ParameterValue(LaunchConfiguration('invert_servo'), value_type=ParameterType.PARAMETER_BOOL),
                'center_on_start':ParameterValue(LaunchConfiguration('center_on_start'), value_type=ParameterType.PARAMETER_BOOL),
                'lost_max_frames':ParameterValue(LaunchConfiguration('lost_max_frames'), value_type=ParameterType.PARAMETER_INTEGER),
                'track_class':    ParameterValue(LaunchConfiguration('track_class'), value_type=ParameterType.PARAMETER_INTEGER),
                'save_frames':    ParameterValue(LaunchConfiguration('save_frames'), value_type=ParameterType.PARAMETER_BOOL),
            }]
        )]
    )

    return LaunchDescription([
        color,
        arg_px2_pkg, arg_px2_exec, arg_px3_pkg, arg_px3_exec,
        arg_camera, arg_hfov, arg_vfov, arg_ctrlmode, arg_kp, arg_kpx, arg_invert, arg_center,
        arg_lost, arg_cls, arg_save,
        arg_serial, arg_baud, arg_gap,
        px3_node, px2_node
    ])

