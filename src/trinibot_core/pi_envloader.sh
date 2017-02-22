#!/usr/bin/env bash


export ROS_IP=192.168.1.6
export ROS_MASTER_URI=http://192.168.1.15:11311
export ROS_HOSTNAME=raspberrypi
echo $ROS_IP
echo $ROS_MASTER_URI
source /home/pi/rosws/devel_isolated/setup.bash
source /home/pi/external-rospacks/devel/setup.bash
source /home/pi/triniBot/devel/setup.bash
sudo modprobe bcm2835-v4l2
#echo "Adding paths to python"
#export PYTHONPATH=$PYTHONPATH:/home/pi/triniBot/tbros/core
workon cv3
#sleep 2
exec "$@"
