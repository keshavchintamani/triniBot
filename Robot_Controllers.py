#!/usr/bin/python

from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor

import time as Time
import atexit
import threading
import random as Random
import Queue
import serial
from sense_hat import SenseHat
from zmq_subscriber import zmqSub
import sys
import picamera
import time
from subprocess import call
import subprocess


#A thread that reads encoder data from Arduino over Serial
class tBEncoderCapture(threading.Thread):

    def __init__(self, threadID, q):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.q = q
        #self.Serial = serial.Serial('/dev/ttyUSB0', 9600);
        Random.random()
        Random.seed()

    def run(self):
        #Get
        while (True):
            #line = self.Serial.readline()
            line = Random.randint(1,10)
            self.q.put(line)
            Time.sleep(0.01)

#A thread that reads encoder data from Arduino over Serial
class tBSensorCapture(threading.Thread):

    def __init__(self, threadID, q, senseHat):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.q = q
        self.senseHat=senseHat

    def run(self):
        #Get
        while (True):
            print("Getting sensors")

#Implementation of Two wheeled robot assuming LEFT motor is 1 and RIGHT Motor is 3
# now includes steering control

# refactor class name to 2WheelRobot
# Left wheel is motor 1
# Right wheel is motor 3
class TwoWheelRobot():
    def __init__(self):
        self.LEFT_MOTOR=1
        self.RIGHT_MOTOR=3
        self.Directions = ["FORWARD", "REVERSE", "LEFT", "RIGHT"]
        self.tBMotors = MotorHatDCMotorController([self.LEFT_MOTOR, self.RIGHT_MOTOR], 0x60)
        self.isRunning=False
        self.lock = threading.Lock()

    def RunMotor(self,speed):
        #print "{}".format(int(speed[0]))
        while(self.isRunning ==True):
            self.tBMotors.runMotor(self.LEFT_MOTOR, speed)
            self.tBMotors.runMotor(self.RIGHT_MOTOR, speed)
            time.sleep(0.01)
        print("Stopping current task...")
        
    def Drive(self,direction, speed):
        try:
            self.Directions.index(direction)
        except ValueError:
            print "Sorry, direction {} is not supported".format(direction)
        self.StopThread()
        if(direction==self.Directions[0]):
            self.tBMotors.setDirection(self.LEFT_MOTOR, "FORWARD")
            self.tBMotors.setDirection(self.RIGHT_MOTOR, "FORWARD")
        elif(direction==self.Directions[1]):
            self.tBMotors.setDirection(self.LEFT_MOTOR, "REVERSE")
            self.tBMotors.setDirection(self.RIGHT_MOTOR, "REVERSE")
        elif(direction==self.Directions[2]):
            self.tBMotors.setDirection(self.LEFT_MOTOR, "REVERSE")
            self.tBMotors.setDirection(self.RIGHT_MOTOR, "FORWARD")
        elif(direction==self.Directions[3]):
            self.tBMotors.setDirection(self.LEFT_MOTOR, "FORWARD")
            self.tBMotors.setDirection(self.RIGHT_MOTOR, "REVERSE")
        self.isRunning= True;
        t=threading.Thread(target=self.RunMotor, args=(speed,))
        print "Thread count: %d, Active: %s" % (threading.active_count(),
                                                threading.current_thread())

        t.start()

    def StopThread(self):
        #Makes an existing while loop in thread exit
        self.isRunning=False
                
        time.sleep(0.01)

    def Stop(self):
        self.StopThread()
        print "Stopping Motors"
        self.tBMotors.stopMotors()
            
    
    def GetIfRunning(self):
        return(self.isRunning)
    
                                 
#Wrapper and helper functions for 4 axis DC motor for Adafruit motorHat
class MotorHatDCMotorController():

    def __init__(self, motors, i2caddress):

        self.mh = Adafruit_MotorHAT(addr=i2caddress)
        allmotors=[1,2,3,4]
        self.user_motors=motors;
        self.motor = {}
        self.motor.fromkeys(allmotors);
        for (i,x) in enumerate(self.user_motors):
            print"Initializing Motor %s" % x
            if (x < 1 or x > 4):
                print "ID out of range"
                return (False)
            self.motor[x] = self.mh.getMotor(x)
            
        #self.sensorq = sensorq
        self.haltRequest = False
        self.speed = 0;
        #TODO
        #Run thread to get encoders from Arduino
        #self.encoderq = Queue.Queue()
        #tBEncoders = tBEncoderCapture("encoder-capture-thread", encoderq)

    def setDirection(self, mId, direction):

        # Set the direction
        if direction == "FORWARD":
            print "Driving Forward Motor %s " % mId
            self.motor[mId].run(Adafruit_MotorHAT.FORWARD)
        elif direction == "REVERSE":
            print "Driving Backward Motor %s " % mId
            self.motor[mId].run(Adafruit_MotorHAT.BACKWARD)
        else:
            print "Invalid direction"
            return (False)

    def runMotor(self, mId, speed):

        self.motor[mId].setSpeed(speed)
        #print "In run motor 2"
        #wheel_speed = self.encoderq.get()
        #print "In run motor 3"
        #print "Speed = %s" % wheel_speed
        #previous_error = 0
        #integral = 0
        #PID controoler when everything is ready
        #while(1)
            #get time 
            #error = setpoint - wheel_speed
            #integral = integral + error*dt
            #derivative = (error - previous_error)/dt
            #output = Kp*error + Ki*integral + Kd*derivative
            #previous_error = error
                
    def stopMotors(self):
        print("Attemping to stop all motors")
        for mId in self.user_motors:
            self.motor[mId].run(Adafruit_MotorHAT.RELEASE)
        
    # recommended for auto-disabling motors on shutdown!
    def turnOffMotor(self, mId):
        print "Releasing motor %s" % mId 
        self.motor[mId].run(Adafruit_MotorHAT.RELEASE)

    def cleanClose():
        self.stopMotors()

