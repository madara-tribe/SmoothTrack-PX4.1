#!/usr/bin/env bash
set -e

ros2 launch px3_launch px3_system_launch.py \
  camera_path:=/dev/video2 \
  serial_port:=/dev/ttyACM0 baud:=9600 \
  lost_max_frames:=200 save_frames:=true \
  tracker_type:=KCF enforce_bgr8:=true \
  thirds_target_:=center draw_thirds_overlay:=true


