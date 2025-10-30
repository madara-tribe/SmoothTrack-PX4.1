sudo apt install udev
whoami
# hagi
# OWNER="h***"
sudo usermod -aG dialout hagi

### USB CAMERA
udevadm info --name=/dev/video2 --attribute-walk

# when to set up
sudo usermod -aG dialout hagi
sudo cp 99-brio100.rules /etc/udev/rules.d/
sudo cp brio100-setup@.service /etc/systemd/system/
sudo udevadm control --reload-rules
sudo systemctl daemon-reload
## re-trigger only video4linux; or unplug/plug camera
sudo udevadm trigger --subsystem-match=video4linux

## verify: symlink should exist and point to the capture node
ls -l /dev/video/brio100
## test
v4l2-ctl -d /dev/video/brio100 --get-fmt-video


### when reset
sudo rm -f /etc/udev/rules.d/99-brio100.rules
sudo rm -f /etc/systemd/system/brio100-setup@.service
sudo udevadm control --reload-rules
sudo systemctl daemon-reload
sudo udevadm trigger --subsystem-match=video4linux
## remove symlink if any
sudo rm -f /dev/video/brio100

