#!/usr/bin/env python

import rospy
import roslib
roslib.load_manifest('balltracker')
from balltracker.msg import ballcoords
import time as Time
#from tbMotorController import TwoWheelRobot
import threading
#from tbAnalogSensors import SPIAnalog
import random

from balltracker.msg import command

def callback(data):
    global ball_X, ball_Radius
    ball_X = data.x
    ball_Radius = data.radius

def CenterBall():

    global ball_X
    if -0.2 <= ball_X <=0.2:
        rospy.loginfo("Ball error: %s", ball_X)
        #Do something with the radius

    else:
        worker_thread = threading.Thread(name = "balltracker_worker", target = lookForBall)
        worker_thread.start()
        worker_thread.join()

def followBall():
    global ball_old, ball_Radius
    factor = 2
    max_speed=200
    if ball_old > ball_Radius:
        publish_command("HIGH", "STOP", 0)
        publish_command("HIGH", "FORWARD", factor*(ball_old-ball_Radius)/ball_Radius*200)
        ball_old = ball_Radius

def lookForBall():
    global ball_X
    total = 2
    for i in range (0, total):
        if -0.2 <= ball_X <=0.2:
            rospy.loginfo("Found ball. exiting search")
            robot.Stop()
            continue
        if (i % 2 == 0):
            Direction = "LEFT"
        else:
            Direction = "RIGHT"
        print "Looking for ball: " + Direction
        publish_command("HIGH", Direction, 80)
        Time.sleep(2)
        publish_command("HIGH", "STOP", 0)

def takeEvasiveAction():
    global sensors
    publish_command("HIGH", "STOP", 0)
    # while(True):
    #     Time.sleep(0.25)
    #     robot.Drive("LEFT", 80)
    #     distance = random.randrange(0, 600, 20)#sensors.readAdc(0)
    #     if (distance > 400):
    #         break

def publish_command(priority, direction, speed):
    global pub
    msg = command()
    msg.direction = direction
    msg.speed = speed
    msg.priority = priority
    pub.publish(msg)

def listener():
    global sensors, pub

    rospy.init_node('balltracker_subscriber', anonymous=True)
    rospy.Subscriber('/tbballtracker/coordinates', ballcoords, callback)
    pub = rospy.Publisher('/tbmotionplanner/command', command, queue_size=10)

    while not rospy.is_shutdown():
        CenterBall()

ball_X=0
ball_old=0
ball_Radius=0

if __name__ == '__main__':
    listener()
    publish_command("LOW", "STOP", 0 )

