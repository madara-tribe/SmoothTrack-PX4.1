#!/usr/bin/env bash
# run px system (px3 flip off)
set -e

ros2 launch px3_launch px3_system_launch.py \
  camera_path:=/dev/video2 \
  serial_port:=/dev/ttyACM0 baud:=9600 min_interval_s:=0.10 \
  hfov_deg:=90.0 vfov_deg:=70.0 kp:=1.0 invert_servo:=true center_on_start:=true \
  lost_max_frames:=200 track_class:=-1 save_frames:=true \
  tracker_type:=KCF enforce_bgr8:=true
