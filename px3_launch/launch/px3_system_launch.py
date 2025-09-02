from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Pretty console colors
    color = SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1')

    # Allow overriding package/executable names at CLI
    arg_px2_pkg  = DeclareLaunchArgument('px2_pkg',  default_value='px2',    description='px2 package')
    arg_px2_exec = DeclareLaunchArgument('px2_exec', default_value='px2',    description='px2 executable')
    arg_px3_pkg  = DeclareLaunchArgument('px3_pkg',  default_value='hw_px3', description='px3 package')
    arg_px3_exec = DeclareLaunchArgument('px3_exec', default_value='hw_px3', description='px3 executable')

    # -------- px2 (camera + control) arguments --------
    arg_camera   = DeclareLaunchArgument('camera_path',     default_value='/dev/video2')
    arg_hfov     = DeclareLaunchArgument('hfov_deg',        default_value='62.0')
    arg_vfov     = DeclareLaunchArgument('vfov_deg',        default_value='48.0')

    # Control (abs/inc)
    arg_ctrlmode = DeclareLaunchArgument('control_mode',    default_value='abs',  description='abs | inc')
    arg_kp       = DeclareLaunchArgument('kp',              default_value='1.0')
    arg_kpx      = DeclareLaunchArgument('kpx_deg_per_px',  default_value='1.0',  description='deg per pixel (inc mode)')
    arg_deadband = DeclareLaunchArgument('deadband_px',     default_value='3',    description='no move if |dx| <= deadband')
    arg_maxstep  = DeclareLaunchArgument('max_step_deg',    default_value='8.0',  description='max deg per update')
    arg_invert   = DeclareLaunchArgument('invert_servo',    default_value='false')
    arg_center   = DeclareLaunchArgument('center_on_start', default_value='true')
    arg_lost     = DeclareLaunchArgument('lost_max_frames', default_value='15')
    arg_cls      = DeclareLaunchArgument('track_class',     default_value='-1')
    arg_save     = DeclareLaunchArgument('save_frames',     default_value='false')

    # Tracker controls
    arg_tracker  = DeclareLaunchArgument('tracker_type',    default_value='KCF',  description='KCF | CSRT | none')
    arg_bgr8     = DeclareLaunchArgument('enforce_bgr8',    default_value='true', description='force BGR8 to tracker')

    # -------- px3 (serial) arguments --------
    arg_serial   = DeclareLaunchArgument('serial_port',     default_value='/dev/ttyACM0')
    arg_baud     = DeclareLaunchArgument('baud',            default_value='9600')
    arg_gap      = DeclareLaunchArgument('min_interval_s',  default_value='0.10')

    # -------- px3 node: opens serial, writes 90°, publishes px3_ready (latched) --------
    px3_node = Node(
        package=LaunchConfiguration('px3_pkg'),
        executable=LaunchConfiguration('px3_exec'),
        name='px3',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'serial_port':     LaunchConfiguration('serial_port'),
            'baud':            ParameterValue(LaunchConfiguration('baud'),            value_type=int),
            'min_interval_s':  ParameterValue(LaunchConfiguration('min_interval_s'),  value_type=float),
        }]
    )

    # -------- px2 node: waits for px3_ready, then DETECT → TRACK --------
    px2_node = TimerAction(
        period=1.0,  # give px3 a moment to come up and publish px3_ready
        actions=[Node(
            package=LaunchConfiguration('px2_pkg'),
            executable=LaunchConfiguration('px2_exec'),
            name='px2',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'device_path':     LaunchConfiguration('camera_path'),
                'hfov_deg':        ParameterValue(LaunchConfiguration('hfov_deg'),        value_type=float),
                'vfov_deg':        ParameterValue(LaunchConfiguration('vfov_deg'),        value_type=float),

                'control_mode':    LaunchConfiguration('control_mode'),  # abs | inc
                'kp':              ParameterValue(LaunchConfiguration('kp'),              value_type=float),
                'kpx_deg_per_px':  ParameterValue(LaunchConfiguration('kpx_deg_per_px'),  value_type=float),
                'deadband_px':     ParameterValue(LaunchConfiguration('deadband_px'),     value_type=int),
                'max_step_deg':    ParameterValue(LaunchConfiguration('max_step_deg'),    value_type=float),
                'invert_servo':    ParameterValue(LaunchConfiguration('invert_servo'),    value_type=bool),
                'center_on_start': ParameterValue(LaunchConfiguration('center_on_start'), value_type=bool),

                'lost_max_frames': ParameterValue(LaunchConfiguration('lost_max_frames'), value_type=int),
                'track_class':     ParameterValue(LaunchConfiguration('track_class'),     value_type=int),
                'save_frames':     ParameterValue(LaunchConfiguration('save_frames'),     value_type=bool),

                'tracker_type':    LaunchConfiguration('tracker_type'),  # KCF | CSRT | none
                'enforce_bgr8':    ParameterValue(LaunchConfiguration('enforce_bgr8'),    value_type=bool),
            }]
        )]
    )

    return LaunchDescription([
        color,
        # args
        arg_px2_pkg, arg_px2_exec, arg_px3_pkg, arg_px3_exec,
        arg_camera, arg_hfov, arg_vfov,
        arg_ctrlmode, arg_kp, arg_kpx, arg_deadband, arg_maxstep,
        arg_invert, arg_center, arg_lost, arg_cls, arg_save,
        arg_tracker, arg_bgr8,
        arg_serial, arg_baud, arg_gap,
        # nodes
        px3_node, px2_node
    ])

