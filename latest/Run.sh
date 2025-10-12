# truch with flip px3 is off
ros2 launch px3_launch px3_system_launch.py \
  camera_path:=/dev/video2 \
  serial_port:=/dev/ttyACM0 baud:=9600 min_interval_s:=0.10 px3_flip:=false \
  hfov_deg:=90.0 vfov_deg:=70.0 kp:=1.0 invert_servo:=false center_on_start:=false \
  lost_max_frames:=15 track_class:=-1 save_frames:=ture \
  tracker_type:=KCF enforce_bgr8:=true

