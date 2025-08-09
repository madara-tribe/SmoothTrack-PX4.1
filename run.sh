ros2 launch px3_launch px3_system_launch.py \
  camera_path:=/dev/video2 \
  serial_port:=/dev/ttyACM0 baud:=9600 min_interval_s:=0.10 \
  hfov_deg:=62.0 vfov_deg:=48.0 kp:=1.0 invert_servo:=false \
  lost_max_frames:=15 track_class:=-1 save_frames:=false
