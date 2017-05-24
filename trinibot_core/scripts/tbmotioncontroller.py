#!/usr/bin/env python

# Ros node for tbmotioncontroller for tracker trinibot with 12T tracks - assume motors have a gear ratio of 150:1

import rospy
import roslib
roslib.load_manifest('trinibot_core')
import sys
import tf_conversions
import time

from tbmotorcontroller import TrackedTrinibot
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import String

wheel_diameter = 39
circumference =	122.5221135
pulses_rotation = 1800
node_name = 'trinibot_motioncontroller'

start_reading = False

def twist_callback(vel):

    start_reading = False
    rospy.loginfo(vel)
    if (vel.linear.x < 0 or vel.linear.x > 0) and (vel.angular.z < 0 or vel.angular.z > 0):
        rospy.logerr("%s cannot execute linear and angular velocities simultaneously")
        return

    if vel.linear.x < 0 or vel.linear.x > 0:
        robot.drive_at_speed(vel.linear.x)
        start_reading = True
    elif vel.angular.z < 0 or vel.angular.z > 0:
        robot.turn_at_rate(vel.angular.z)
        start_reading = True

def stop_callback(cb):
    if cb.data == "STOP":
        robot.stop()
        start_reading = False

def listener():
    global robot, start_reading
    rospy.init_node(node_name, anonymous=True)
    rospy.Subscriber('trinibot/Twist', Twist , twist_callback)
    rospy.Subscriber('trinibot/String', String, stop_callback)

    xy = 0
    robot = TrackedTrinibot(xy, sys.argv[1])
    #Set gains to 10, 1, 1
    robot.setgains(10, 1, 1)
    while not rospy.is_shutdown():
        if(robot.is_running()):
            res = robot.get_feedback();
            rospy.loginfo("feedback %s", res)
            time.sleep(0.01)
        # get the gyro values

if __name__ == '__main__':

    listener()
    robot.exit()

