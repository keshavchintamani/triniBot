#!/usr/bin/python

# Simple DC motor controller script for Adafruit motor hat
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor

import time as Time
import atexit
import threading


class triniBotMotorController(threading.Thread):

    def __init__(self, motors, q):

        threading.Thread.__init__(self)
        self.mh = Adafruit_MotorHAT(addr=0x60)
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
            
        self.q = q
        self.haltRequest = False

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

        print "In run motor"
        self.motor[mId].setSpeed(speed)
        print "In run motor 2"
        wheel_speed = self.q.get()
        print "In run motor 3"
        print "Speed = %s" % wheel_speed
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





