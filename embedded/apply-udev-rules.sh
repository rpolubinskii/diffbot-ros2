#!/bin/sh

sudo ln -sf /dev/null /etc/udev/rules.d/60-ros-humble-rplidar-ros.rules
sudo rm /etc/udev/rules.d/99-ros.rules
sudo cp 99-ros.rules /etc/udev/rules.d
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty --action=add
sleep 1
ls -l /dev/rplidar /dev/imu-encoder-node /dev/motor-controller
