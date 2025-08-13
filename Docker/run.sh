#!/bin/bash

# Path to serial device on host
DEVICE=/dev/cu.usbserial-DK0FJVDT


# Run Docker container
docker run -it --rm \
  --privileged \
  --device=${DEVICE}:${DEVICE} \
  -v /Users/hagi/downloads/place/:/ros2_ws \
  --name px3-container \
  test:latest

# v4l2-ctl --list-devices
