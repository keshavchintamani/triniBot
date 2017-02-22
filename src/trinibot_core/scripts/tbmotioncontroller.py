#!/unsr/bin/env python

import rospy
import roslib
roslib.load_manifest('trinibot_core')

import time as Time
from tbmotorcontroller import TrackedTrinibot
import random

from geometry_msgs.msg import TwistStamped

#This callback receives command updates from other triniBot nodes
def command_callback(data):
    #Process command
    global robot
    #This callback receives pose updates from the sensehat
    if(data.header.frame_id=="LINEAR"):
        robot.drive_to_distance( data.twist.linear.x)
    elif(data.header.frame_id=="ANGULAR"):
        robot.turn_to_angle(data.twist.angular.z)
    elif(data.header.frame_id=="STOP"):
        robot.stop()


def listener():

    rospy.init_node('motioncontroller_subscriber', anonymous=True)
    rospy.Subscriber('/tbmotionplanner/TwistStamped', TwistStamped, command_callback)

    rate= rospy.Rate(1)
    while not rospy.is_shutdown():
        #If an obstacle is detected
        rate.sleep()

robot = TrackedTrinibot("/dev/ttyACM0")

if __name__ == '__main__':
    listener()
    robot.Stop()

