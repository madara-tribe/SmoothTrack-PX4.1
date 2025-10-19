sudo apt install udev
whoami
# hagi
# OWNER="h***"
sudo usermod -aG dialout hagi

### USB CAMERA
udevadm info --name=/dev/video2 --attribute-walk


sudo usermod -aG dialout hagi
sudo cp 99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

### when uninstall
sudo rm /etc/udev/rules.d/99-realsense-libusb.rules
sudo rm /etc/udev/rules.d/99-arduino.rules
sudo udevadm control --reload-rules 
sudo udevadm trigger

sudo usermod -aG dialout hagi
sudo cp 99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

