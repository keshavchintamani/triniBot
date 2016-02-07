#!/usr/bin/env python

import rospy
import roslib
import random
import time as Time

roslib.load_manifest('balltracker')

from balltracker.msg import command

def publish_command(priority, direction, speed):
    global pub
    msg = command()
    msg.direction = direction
    msg.speed = speed
    msg.priority = priority
    pub.publish(msg)

def listener():
    global pub

    rospy.init_node('balltracker_subscriber', anonymous=True)
    pub = rospy.Publisher('/tbmotionplanner/command', command, queue_size=10)

    directions = ["FORWARD", "REVERSE", "LEFT", "RIGHT", "STOP"]
    priority =["LOW", "MEDIUM", "HIGH"]
    speed =150
    while not rospy.is_shutdown():
        for i in directions:
            for j in priority:
		print("{}:{}").format(i,j)
                publish_command(j, i, random.randint(0, speed) )
                Time.sleep(2)

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
	pass
