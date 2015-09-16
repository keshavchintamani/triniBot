#!/usr/bin/python

# Simple DC motor controller script for Adafruit motor hat
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor

import time as Time
import atexit
import threading


class triniBotMotorController(threading.Thread):

    def __init__(self, motorId, q):

        threading.Thread.__init__(self)
        self.motorId = motorId
        if (self.motorId < 1 or self.motorId > 4):
            print "ID out of range"
            return (False)


        self.mh = Adafruit_MotorHAT(addr=0x60)
        self.q = q
        self.haltRequest == False

    def initialize(self, i2cadd):
        print "Starting motor %s" % self.motorId
        self.motor = self.mh.getMotor(self.motorId - 1)

    def setDirection(self, direction):

        # Set the direction
        if direction == "FORWARD":
            # print "Driving Forward"
            self.motor.run(Adafruit_MotorHAT.FORWARD)
        elif direction == "REVERSE":
            # print "Driving Backward"
            self.motor.run(Adafruit_MotorHAT.BACKWARD)
        else:
            # print "Invalid direction"
            return (False)

    def runMotor(self, speed):

        if (speed < 0 or speed > 255):
            print "Invaid speed!"
            return (False)

        print("Running at " + repr(speed))
        pwm = 0
        while(self.haltRequest == True):
                for x in range(0, 255):
                    self.motor.setSpeed(x)
                    wheel_speed = self.q.get()
                    if wheel_speed > speed

    def stopMotor(self, mID):
        print("Attemping to stop " + repr(mID))
        self.motor.run(Adafruit_MotorHAT.RELEASE)
        self.haltRequest = True

    # recommended for auto-disabling motors on shutdown!
    def turnOffMotors(self):
        self.motor.run(Adafruit_MotorHAT.RELEASE)

    def run(self):
        wheel_speed = self.q.get()
        speed = 0;
        #self.setDirection("FORWARD")
        self.runMotor(100)




