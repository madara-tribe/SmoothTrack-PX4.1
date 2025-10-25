from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Pretty console colors
    color = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")

    # Allow overriding package/executable names at CLI
    arg_px2_pkg  = DeclareLaunchArgument("px2_pkg",  default_value="px2",    description="px2 package")
    arg_px2_exec = DeclareLaunchArgument("px2_exec", default_value="px2",    description="px2 executable")
    arg_px3_pkg  = DeclareLaunchArgument("px3_pkg",  default_value="hw_px3", description="px3 package")
    arg_px3_exec = DeclareLaunchArgument("px3_exec", default_value="hw_px3", description="px3 executable")

    # -------- px2 (camera + control) arguments --------
    arg_camera   = DeclareLaunchArgument("camera_path",     default_value="/dev/video2")

    # ABS control only
    arg_lost     = DeclareLaunchArgument("lost_max_frames", default_value="15")
    arg_save     = DeclareLaunchArgument("save_frames",     default_value="false")

    # Tracker controls
    arg_tracker  = DeclareLaunchArgument("tracker_type",    default_value="KCF",  description="KCF | CSRT | none")
    arg_bgr8     = DeclareLaunchArgument("enforce_bgr8",    default_value="true", description="force BGR8 to tracker")
    arg_thirds   = DeclareLaunchArgument("thirds_target_",       default_value="center", description="left|center|right|auto")
    arg_overlay  = DeclareLaunchArgument("draw_thirds_overlay", default_value="true",   description="draw 3x3 grid overlay")

    # -------- px3 (serial) arguments --------
    arg_serial   = DeclareLaunchArgument("serial_port",     default_value="/dev/ttyACM0")
    arg_baud     = DeclareLaunchArgument("baud",            default_value="9600")
    arg_center_deg   = DeclareLaunchArgument("center_deg",         default_value="90.0",  description="Servo center (deg).")

    # -------- px3 node: opens serial, writes 90°, publishes px3_ready (latched) --------
    px3_node = Node(
        package=LaunchConfiguration("px3_pkg"),
        executable=LaunchConfiguration("px3_exec"),
        name="px3",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "serial_port":     LaunchConfiguration("serial_port"),
            "baud":            ParameterValue(LaunchConfiguration("baud"), value_type=int),
            "center_deg":               ParameterValue(LaunchConfiguration("center_deg"), value_type=float),
        }]
    )

    # -------- px2 node: waits for px3_ready, then DETECT → (optional) TRACK --------
    px2_node = TimerAction(
        period=1.0,  # give px3 a moment to publish px3_ready (latched)
        actions=[Node(
            package=LaunchConfiguration("px2_pkg"),
            executable=LaunchConfiguration("px2_exec"),
            name="px2",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "device_path":     LaunchConfiguration("camera_path"),
                "lost_max_frames": ParameterValue(LaunchConfiguration("lost_max_frames"), value_type=int),
                "save_frames":     ParameterValue(LaunchConfiguration("save_frames"),     value_type=bool),
                "tracker_type":    LaunchConfiguration("tracker_type"),  # KCF | CSRT | none
                "enforce_bgr8":    ParameterValue(LaunchConfiguration("enforce_bgr8"),    value_type=bool),
                "thirds_target_":       LaunchConfiguration("thirds_target_"),
                "draw_thirds_overlay": ParameterValue(LaunchConfiguration("draw_thirds_overlay"), value_type=bool),
            }]
        )]
    )

    return LaunchDescription([
        color,
        # arg declarations
        arg_px2_pkg, arg_px2_exec, arg_px3_pkg, arg_px3_exec,
        
        # px2 args
        arg_camera, arg_lost, arg_save, arg_tracker, arg_bgr8, arg_thirds, arg_overlay,

        # px3 args
        arg_serial, arg_baud, arg_center_deg,
       
        # nodes
        px3_node, px2_node
    ])