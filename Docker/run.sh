# bin/bash
BRIO_DEV="/dev/video2"
ARDUINO_DEV="/dev/ttyACM0"
# Check if Arduino and CAM device exists
if [[ -z "${BRIO_DEV:-}" ]]; then
  echo "ERROR: BRIO_DEV is empty. Set BRIO_DEV=/dev/video2 (or /dev/video/brio100) and retry."
  exit 1
fi
if [[ ! -e "$BRIO_DEV" ]]; then
  echo "ERROR: '$BRIO_DEV' not found on host. Check camera and device path."
  exit 1
fi

if [ ! -e "$ARDUINO_DEV" ]; then
  echo "Error: $ARDUINO_DEV not found on host."
  exit 1
fi
xhost +local:docker

docker run -it --rm \
  --net=host \
  --privileged \
  --env="DISPLAY=$DISPLAY" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --device="${BRIO_DEV}:/dev/video/brio100" \
  --group-add video \
  -v /home/hagi/Downloads/place/:/ros2_ws \
  test:latest
  #--device=/dev/video-cam:$CAMERA_DEV \
  #--group-add video \
# v4l2-ctl --list-devices
