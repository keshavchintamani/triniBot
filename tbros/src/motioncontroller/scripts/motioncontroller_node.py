#!/usr/bin/env python

import rospy
import roslib
roslib.load_manifest('balltracker')

import time as Time
from tbMotorController import TwoWheelRobot
import threading
from tbanalogsensors import SPIAnalog
import random

#import messages
from balltracker.msg import command
from sensor_msgs.msg import Imu

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

#This callback receives command updates from other triniBot nodes
def command_callback(data):
    #Process command
    global robot
    #rospy.loginfo("Received %s", data)
    #rospy.loginfo("Received command: %s",data)
    if data.direction == "STOP":
        robot.Stop()
    else:
        worker_thread = threading.Thread(name = "motorcontroller_worker", target = robot.drive, args = [str(data.direction), int(data.speed), float(data.angle)])
        worker_thread.start()
        worker_thread.join()

#This callback receives pose updates from the sensehat
def pose_callback(data):
    #rospy.loginfo("Received angle %s", data.orientation.z)
    #rospy.loginfo("Received from SenseHat: %s", data)
    robot.setPose(data.angular_velocity.z)
    #angular_velocity.z
    #data.orientation.z

def avoid_collisions():

    global sensors
    obstacle_distance = 0
	 #sensors.readAdc(0)
    if obstacle_distance >= 200:
	robot.Stop()
    rospy.loginfo("Obstacle at: %s", obstacle_distance)
    global sensors, robot
    #obstacle_distance = sensors.readAdc(0)
    #if obstacle_distance > 200:
	#robot.Stop()
	#rospy.loginfo("Obstacle at: %s", obstacle_distance)

def listener():

    rospy.init_node('motioncontroller_subscriber', anonymous=True)
    rospy.Subscriber('/tbmotionplanner/command', command, command_callback)
    rospy.Subscriber('/sensehat/pose', Imu, pose_callback)

    rate= rospy.Rate(1)
    while not rospy.is_shutdown():
        #If an obstacle is detected
        rate.sleep()

robot = TwoWheelRobot()
#sensors = SPIAnalog()


if __name__ == '__main__':
    listener()
    robot.Stop()

