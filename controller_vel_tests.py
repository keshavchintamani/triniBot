#!/usr/bin/python

#Simple DC motor controller script for Adafruit motor hat 
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor

import time as Time
import atexit
import serial


# create a default object, no changes to I2C address or frequency
mh = Adafruit_MotorHAT(addr=0x60)

# there are four motors
motors=4
motor=[]
serialPort = serial.Serial('/dev/ttyUSB0',9600)

motor_status = [];
def Initialize(i2cadd):
	
    print ("Starting triniBot")
    print ("Number of motors=" + repr(motors));
    for x in range(0,motors):
        print("Initializing Motor " + repr(x))
        motor.append(mh.getMotor(x+1))
	    # set the speed to start, from 0 (off) to 255 (max speed)
        motor[x].setSpeed(150)
        motor[x].run(Adafruit_MotorHAT.FORWARD)
        Time.sleep(0.5)
        		

def runMotor(mID, speed, direction):

    #motor_status[mID] = True;
    if (mID < 1 or mID > motors):
        print "ID out of range"
        return (False)

    if (speed < 0 or speed > 255):
        print "Invaid speed!"
        return  (False)

    #Set the direction
    if direction == "FORWARD":
        #print "Driving Forward"
        motor[mID].run(Adafruit_MotorHAT.FORWARD)
    elif direction == "REVERSE":
        #print "Driving Backward"
        motor[mID].run(Adafruit_MotorHAT.BACKWARD)
    else:
        #print "Invalid direction"
        return (False)
	
    #while (True):
    #print (speed)
    motor[mID].setSpeed(speed)
        

def stopMotor(mID):
    print("Attemping to stop " +repr(mID))
    motor[mID].run(Adafruit_MotorHAT.RELEASE)
    

# recommended for auto-disabling motors on shutdown!
def turnOffMotors():
    for i,val in enumerate(motor):
        val.run(Adafruit_MotorHAT.RELEASE)

atexit.register(turnOffMotors)

#Run tests
#speeds = [10, 50, 100, 200, 255]
Initialize(1)

#for i, speed in enumerate(speeds):
speed = 10      
for i in range (1,250):
    runMotor(1, speed, "FORWARD")
    #line = serialPort.readline()
    print( serialPort.readline() + "\t" + str(speed) )
    Time.sleep(0.01)
    #print("Stopping motor... next test...") 
    #turnOffMotors()



	
