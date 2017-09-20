#!/bin/bash
export LC_ALL="C"
source /opt/ros/kinetic/setup.bash
source /home/kc/sandbox/external-rospacks/devel/setup.bash
#export ROSLAUNCH_SSH_UNKNOWN=1
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/home/kc/sandbox/external-rospacks/src/ORB_SLAM2/Examples/ROS
#export ROS_IP=kc.local
export ROS_MASTER_URI=http://kc-RPi3-mate.local:11311
#export ROS_HOSTNAME=kc
#echo $ROS_IP
echo $ROS_MASTER_URI

exec "$@"

