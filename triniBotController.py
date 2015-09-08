#!/usr/bin/python

#Simple DC motor controller script for Adafruit motor hat 
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor

import time as Time
import atexit
import serial

class triniBotController:

    def __init__(self, noofmotors):
        self.motor=[]
        self.motors=noofmotors
        self.serialPort = serial.Serial('/dev/ttyUSB0', 9600);
        self.motor_status = [];
        self.mh = Adafruit_MotorHAT(addr=0x60)

    def initialize(self, i2cadd):
        print ("Starting triniBot")
        for x in range(0, self.motors):
            print("Initializing Motor " + repr(x))
            self.motor.append(self.mh.getMotor(x+1))
	        # set the speed to start, from 0 (off) to 255 (max speed)
            self.motor[x].setSpeed(150)
            self.motor[x].run(Adafruit_MotorHAT.FORWARD)
            Time.sleep(0.5)
        		
    def setSpeed(self, mID, direction):
        if (mID < 1 or mID > self.motors):
        print "ID out of range"
        return (False)
        #Set the direction
        if direction == "FORWARD":
            #print "Driving Forward"
            self.motor[mID].run(Adafruit_MotorHAT.FORWARD)
        elif direction == "REVERSE":
            #print "Driving Backward"
            self.motor[mID].run(Adafruit_MotorHAT.BACKWARD)
        else:
            #print "Invalid direction"
            return (False)

      

    def runMotor(self, mID, speed, direction):

        #motor_status[mID] = True;
        if (mID < 1 or mID > self.motors):
            print "ID out of range"
            return (False)

        if (speed < 0 or speed > 255):
            print "Invaid speed!"
            return  (False)
	    
        aspeed=0
        dc=0
        while (aspeed <= speed):
            print("Running at " + repr(speed))
            dc += 20
            self.motor[mID].setSpeed(dc)
            #get encoder 
            aspeed = getActualSpeed()
            

    def stopMotor(self, mID):
        print("Attemping to stop " +repr(mID))
        motor[mID].run(Adafruit_MotorHAT.RELEASE)
    

    # recommended for auto-disabling motors on shutdown!
    def turnOffMotors(self):
        for i,val in enumerate(self.motor):
            val.run(Adafruit_MotorHAT.RELEASE)
