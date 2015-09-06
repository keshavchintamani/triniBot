#!/usr/bin/python

#Simple DC motor controller script for Adafruit motor hat 
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor

import time
import atexit

# create a default object, no changes to I2C address or frequency
mh = Adafruit_MotorHAT(addr=0x60)

motors=2
motor=[]

motor_status = [];
def Initialize(i2cadd):
	
    for x in range(0,motors):
        motor.append(mh.getMotor(x+1))
	#initialize motors by driving them a little bit (seems needed)	
    for x in range(0,motors):
	    # set the speed to start, from 0 (off) to 255 (max speed)
        motor[x].setSpeed(150)
        motor[x].run(Adafruit_MotorHAT.FORWARD)
        #motor[x].run(Adafruit_MotorHAT.RELEASE)
		

def runMotor(mID, speed, direction):

    
    print (mID)
    print (motor[mID])
    #motor_status[mID] = True;
    if (mID < 1 or mID > motors):
        print "ID out of range"
        return (False)

    if (speed < 0 or speed > 255):
        print "Invaid speed!"
        return  (False)

    #Set the direction
    if direction == "FORWARD":
        print "Driving Forward"
        motor[mID].run(Adafruit_MotorHAT.FORWARD)
    elif direction == "REVERSE":
        print "Driving Backward"
        motor[mID].run(Adafruit_MotorHAT.BACKWARD)
    else:
        print "Invalid direction"
        return (False)
	
    while (True):
        #print (speed)
        motor[mID].setSpeed(speed)
        

def stopMotor(mID):
	print(mID + " stop")

# recommended for auto-disabling motors on shutdown!
def turnOffMotors():
    for i,val in enumerate(motor):
        val.run(Adafruit_MotorHAT.RELEASE)

atexit.register(turnOffMotors)

Initialize(1)
runMotor(1, 100, "REVERSE")


#for x in range(1,3):
#        for y in range(255):
 #               runMotor(x, y, "REVERSE")
  #              time.sleep(0.01)


	
