#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from trinibot_msgs.msg import ballcoords
import time as Time
import threading

ball_X=0
ball_Y=0
ball_Radius=0

def callback(data):
    global ball_X
    ball_X = data.x
    ball_Y = data.y
    ball_Radius = data.radius
    rospy.loginfo(rospy.get_caller_id()+ "Received %s", data.x)

def CenterBall():

    global ball_X
    if not -0.1 <= ball_X <= 0.1:
        for i in range(0,2):
            print "Turning left"
            Time.sleep(1)
        for i in range(0,2):
            print "Turning right"
            Time.sleep(1)

def listener():

    rospy.init_node('balltracker_subscriber', anonymous=True)
    rospy.Subscriber('coordinates', ballcoords, callback)

    while not rospy.is_shutdown():
            CenterBall()

    #rospy.spin()

if __name__ == '__main__':
    listener()



