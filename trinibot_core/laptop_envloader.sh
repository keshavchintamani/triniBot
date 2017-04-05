#!/usr/bin/env bash


export ROS_IP=192.168.1.15
export ROS_MASTER_URI=http://192.168.1.15:11311
export ROS_HOSTNAME=kc
echo $ROS_IP
echo $ROS_MASTER_URI
source /usr/local/bin/virtualenvwrapper.sh
source /opt/ros/kinetic/setup.bash
source /home/kc/sandbox/external-rospacks/devel/setup.bash
#workon trinibot-dev
exec "$@"

