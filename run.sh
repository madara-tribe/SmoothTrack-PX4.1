#!/usr/bin/env bash
# run px system (px3 flip off)
set -e

ros2 launch px3_launch px3_system_launch.py \
  camera_path:=/dev/video2 \
  serial_port:=/dev/ttyACM0 baud:=9600 max_angle:=200.0 \
  min_angle:=15.0 center_on_start:=true invert_angle:=true \
  lost_max_frames:=200 save_frames:=true \
  tracker_type:=KCF enforce_bgr8:=true
