#!/usr/bin/env bash
set -e

ros2 launch px3_launch px3_system_launch.py \
  device_path:=/dev/video2 \
  serial_port:=/dev/ttyACM0 \
  lost_max_frames:=5 \
  save_frames:=true \
  enforce_bgr8:=false \
  thirds_target_:=auto \
  preprocess_enable:=false


