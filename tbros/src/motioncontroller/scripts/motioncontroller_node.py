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

#Emulator class for the robot
class XXXWheelRobot():
    def __init__(self):
        self.LEFT_MOTOR=1
        self.RIGHT_MOTOR=3
        self.Directions = ["FORWARD", "REVERSE", "LEFT", "RIGHT"]
        #self.tBMotors = MotorHatDCMotorController([self.LEFT_MOTOR, self.RIGHT_MOTOR], 0x60)
        self.isRunning=False
        self.lock = threading.Lock()

    def RunMotor(self,speed, direction):
        print "{} at {}".format(direction,speed)
        Time.sleep(0.1)

    def Drive(self,direction, speed):
        self.RunMotor(speed,direction)

    def StopThread(self):
        #Makes an existing while loop in thread exit
        self.isRunning=False
        Time.sleep(0.01)

    def Stop(self):
        self.StopThread()
        print "Stopping Motors"
        #self.tBMotors.stopMotors()

def callback(data):
    #Process command
    rospy.loginfo("Received %s", data)

    if data.direction == "STOP":
        robot.Stop()
    else:
        worker_thread = threading.Thread(name = "motorcontroller_worker", target = robot.Drive, args = [str(data.direction), int(data.speed)])
        worker_thread.start()
        worker_thread.join()

def avoid_collisions():
    global sensors
    obstacle_distance = sensors.readAdc(0)
    if obstacle_distance >= 200:
	robot.Stop()
    rospy.loginfo("Obstacle at: %s", obstacle_distance)

def listener():

    rospy.init_node('motioncontroller_subscriber', anonymous=True)
    rospy.Subscriber('/tbmotionplanner/command', command, callback)

    while not rospy.is_shutdown():
        print("Waiting for commands")
        #If an obstacle is detected
        avoid_collisions()
        Time.sleep(1)

robot = TwoWheelRobot()
sensors = SPIAnalog()
if __name__ == '__main__':
    listener()
    robot.Stop()

