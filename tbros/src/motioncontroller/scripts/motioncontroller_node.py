#!/usr/bin/env python

import rospy
import roslib
roslib.load_manifest('balltracker')

import time as Time
from tbMotorController import TwoWheelRobot
import threading
from tbAnalogSensors import SPIAnalog
import random

#import messages
from balltracker.msg import command

def callback(data):
    #Process command
    global robot
    rospy.loginfo("Received %s", data)

    if data.direction == "STOP":
        robot.Stop()
    else:
        worker_thread = threading.Thread(name = "motorcontroller_worker", target = robot.Drive, args = [str(data.direction), int(data.speed)])
        worker_thread.start()
        worker_thread.join()

def avoid_collisions():
    global sensors, robot
    obstacle_distance = sensors.readAdc(0)
    if obstacle_distance > 200:
	robot.Stop()
	rospy.loginfo("Obstacle at: %s", obstacle_distance)

def listener():

    rospy.init_node('motioncontroller_subscriber', anonymous=True)
    rospy.Subscriber('/tbmotionplanner/command', command, callback)
    rate= rospy.Rate(60)
    while not rospy.is_shutdown():
        print("Waiting for commands")
        #If an obstacle is detected
        #avoid_collisions()
        rate.sleep()

robot = TwoWheelRobot()
sensors = SPIAnalog()
if __name__ == '__main__':
    listener()
    robot.Stop()

