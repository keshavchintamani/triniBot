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
    while not rospy.is_shutdown():
        for i in directions:
            publish_command("LOW", i, random.randint(0, 255) )
            Time.sleep(2)
        for i in directions:
            publish_command("MEDIUM", i, random.randint(0, 255) )
            Time.sleep(2)
        for i in directions:
            publish_command("HIGH", i, random.randint(0, 255) )
            Time.sleep(2)

if __name__ == '__main__':
    listener()
    publish_command("LOW", "STOP", 0 )
