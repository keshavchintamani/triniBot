#! /bin/bash

source /opt/ros/kinetic/setup.bash
source /home/keshavchintamani/trinibot_ws/devel/setup.bash
export ROS_MASTER_URI=http://kc-RPi3-mate.local:11311
roscore
