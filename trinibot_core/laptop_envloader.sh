#!/usr/bin/env bash

#export MYROSWS=/home/pi/triniBot/tbros
#export MYEXTERNALROSWS=/external-rospacks
export ROS_MASTER_URI=http://192.168.1.15:11311
export ROS_HOSTNAME=kc
source /opt/ros/kinetic/setup.bash
source ~/sandbox/external_rospacks/ORB_SLAM2/Examples/ROS/ORB_SLAM2/build/devel/setup.bash
source /home/kc/sandbox/external-rospacks/devel/setup.bash
exec "$@"
