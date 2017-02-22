#!/usr/bin/env python
import platform
import atexit

#Very Pi imports
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor
#from tbAnalogSensors.py import SPIAnalog
import time as Time
import atexit
import threading
import sys
import time

import serial
import logging
import re
import signal

#Plot include
import numpy as np
#Signal handler to kill active threads
class SignalHandler:

    stopper = None
    workers = None

    def __init__(self, stopper, workers):
        self.stopper = stopper
        self.workers = workers

    def __call__(self, signum, frame):
        self.stopper.set()
        self.bot.Stop()
        #for worker in self.workers:
        #   worker.join()
        logger.info("Ctrl-C pressed so exiting...")
        sys.exit(0)

    def SetRobot(self, bot):
        self.bot = bot

#Lowest level wrapper around MotorHat code from Adafruit.
# A class to implement a controller for 1 to 4 DC motors at a given I2C address.
# Works only with the Adafruit motorhat library...as
class MotorHatDCMotorController():

    def __init__(self, motors, i2caddress):
        global Adafruit_MotorHat
        self.mh = Adafruit_MotorHAT(addr=i2caddress)
        allmotors=[1,2,3,4]
        self.user_motors=motors
        self.motor = {}
        self.motor.fromkeys(allmotors)
        for (i,x) in enumerate(self.user_motors):
            if (x < 1 or x > 4):
                logger.error("motor ID out of range")
                return (False)
            self.motor[x] = self.mh.getMotor(x)

    def setDirection(self, mId, direction):

        # Set the direction
        if direction == "FORWARD":
            self.motor[mId].run(Adafruit_MotorHAT.FORWARD)
        elif direction == "REVERSE":
            self.motor[mId].run(Adafruit_MotorHAT.BACKWARD)
        else:
            logger.error("Invalid direction provided")
            return (False)

    def runMotor(self, mId, dutycycle):
        self.motor[mId].setSpeed(dutycycle)

    def stopMotors(self):
        print("Attemping to stop all motors")
        for mId in self.user_motors:
            self.motor[mId].run(Adafruit_MotorHAT.RELEASE)

    # recommended for auto-disabling motors on shutdown!
    def stopMotor(self, mId):
        print "Releasing motor %s" % mId
        self.motor[mId].run(Adafruit_MotorHAT.RELEASE)

    def cleanClose():
        self.stopMotors()

class tBSerialReader():

    def __init__(self, devid):

        self.devid = devid
        self.old_encoder_left=0
        self.old_encoder_right=0
        self._startserial()

    def _startserial(self):

        print("Trying to open Serial arduino: %s ", self.devid)
        try:
            self.Serial = serial.Serial(self.devid, 9600)
            time.sleep(1)
            pass
        except (IOError, ValueError):
            print("Cannot open serial device: %s", self.devid)
            raise IOError
    
    def readserial(self):

        line = self.Serial.readline()
        res = re.findall("[-+]?\d+[\.]?\d*", line)
        try:
            if len(res) > 1:
                self.old_encoder_left  = int(res[0])
                self.old_encoder_right = int(res[1])
                return ((self.old_encoder_left, self.old_encoder_right))
        except ValueError:
            logger.error("Value error exception parsing serial data")
            #return((NoneType, NoneType))
            pass 
    #    return ((self.old_encoder_left, self.old_encoder_right))

    def writeserial(self, message):
        try:
            self.Serial.write(message);
            return(True);
        except (IOError, ValueError):
            print("Cannot open serial device: %s", self.devid)
            raise IOError
        return(False);

#Documentation
#TrackedTriniBot is a specialized tracked platform using the 150:1 micrometal gearmotors
#It assumes certain dimensions of the wheels for its odometry
#Changes to the wheels or transmision should be reflected in the parameters
class TrackedTrinibot():
    global logger


    def __init__(self, serial = "/dev/ttyUSB0"):

        self.Ki = 0.0
        self.Kp = 0.01
        self.Kd = 0.01
        self.logfileindex=0
        self.serial = tBSerialReader(serial)
        self.logger = logging.getLogger('encoder')
        hdlr = logging.FileHandler('/var/tmp/TrackedTrinibot.log')
        formatter = logging.Formatter('%(asctime)s %(levelname)s %(message)s')
        hdlr.setFormatter(formatter)
        self.logger.addHandler(hdlr)
        self.logger.setLevel(logging.DEBUG)
        sh = logging.StreamHandler(sys.stdout)
        sh.setFormatter(formatter)
        sh.setLevel(logging.DEBUG)
        self.logger.addHandler(sh)


    def drive_at_speed(self, target_speed=300.0):
       command = "speed"+ " " + str(target_speed)
       self.logger.info(command);
       self.serial.writeserial(command)
       return(True) 

    def drive_to_distance(self, target_distance=188.49):
       command = "goto"+ " " + str(target_distance)
       self.logger.info(command);
       self.serial.writeserial(command)
       return(True)

    def turn_to_angle(self, target_angle=45):
       command = "turn"+ " " + str(target_angle)
       self.logger.info(command);
       self.serial.writeserial(command)
       return(True)

    def stop(self):
       command = "stop"
       self.serial.writeserial(command)
       return(True)

    def setKp(self, kp):
       self.Kp = float(kp)
       logger.info("Kp=%f", self.Kp)

    def setKi(self, ki):
       self.Ki = float(ki)
       logger.info("Ki=%f", self.Ki)

    def setKd(self, kd):
       self.Kd = float(kd)
       logger.info("Kd=%f", self.Kd)

def main_automation():
    global myrobot, logger

    logger = logging.getLogger('encoder')
    hdlr = logging.FileHandler('/var/tmp/encoders.log')
    formatter = logging.Formatter('%(asctime)s %(levelname)s %(message)s')
    hdlr.setFormatter(formatter)
    logger.addHandler(hdlr)
    logger.setLevel(logging.DEBUG)
    sh = logging.StreamHandler(sys.stdout)
    sh.setFormatter(formatter)
    sh.setLevel(logging.DEBUG)
    logger.addHandler(sh)

    #Initialize the stopper
    Stopper = threading.Event()
    Stopper.clear()
    threads = []

    myrobot = TrackedTrinibot(Stopper, robot_data, encoder_dev= "/dev/ttyACM0")
    threads.append(myrobot)

    handler = SignalHandler(Stopper, threads)
    handler.SetRobot(myrobot)
    signal.signal(signal.SIGINT, handler)

    KP=[0.05]
    KD=[-0.001,0.0, 0.001, 0.01]#]# 0.5, 0.6, 0.7, 0.8, 0.9]
    KI=[-0.001,0.0, 0.001, 0.01]##
    for Kp in KP:
        for Kd in KD:
            for Ki in KI:
                myrobot.setKp(Kp)
                myrobot.setKd(Kd)
                myrobot.setKi(Ki)
                status = myrobot.turn_to_angle(90)
                response = raw_input("Hit enter for next run")
                logger.info(status)#time.sleep(10)
    myrobot.Cleanup()

def main_cli_args():

    global myrobot, logger
    logger = logging.getLogger('encoder')
    hdlr = logging.FileHandler('/var/tmp/encoders.log')
    formatter = logging.Formatter('%(asctime)s %(levelname)s %(message)s')
    hdlr.setFormatter(formatter)
    logger.addHandler(hdlr)
    logger.setLevel(logging.DEBUG)
    sh = logging.StreamHandler(sys.stdout)
    sh.setFormatter(formatter)
    sh.setLevel(logging.DEBUG)
    logger.addHandler(sh)

    #Initialize the stopper
    Stopper = threading.Event()
    Stopper.clear()
    threads = []

    myrobot = TrackedTrinibot(Stopper, robot_data, "/dev/ttyACM0")
    threads.append(myrobot)

    handler = SignalHandler(Stopper, threads)
    handler.SetRobot(myrobot)
    signal.signal(signal.SIGINT, handler)
    if sys.argv[1] == 'd' and len(sys.argv) == 6:
        myrobot.setKp(float(sys.argv[3]))
        myrobot.setKd(float(sys.argv[4]))
        myrobot.setKi(float(sys.argv[5]))
        logger.info("distance: %d", float(sys.argv[2]))
        myrobot.drive_to_distance( float(sys.argv[2]))
    elif sys.argv[1] == 's' and len(sys.argv) == 6:
        myrobot.setKp(float(sys.argv[3]))
        myrobot.setKd(float(sys.argv[4]))
        myrobot.setKi(float(sys.argv[5]))
        myrobot.drive_at_speed( float(sys.argv[2]))
    elif sys.argv[1] == 'a':
        myrobot.setKp(float(sys.argv[3]))
        myrobot.setKd(float(sys.argv[4]))
        myrobot.setKi(float(sys.argv[5]))
        myrobot.turn_to_angle( float(sys.argv[2]))
    else:
        logger.info("Invalid number of arguments")
   
    myrobot.Cleanup()

def robot_data(val):
    logger.info("%1.1f %1.1f %1.1f 1.1%f", val[0], val[1], val[2], val[3])

def exithandler():

    logger.info("Goodbye!")

if __name__ == '__main__':
    atexit.register(exithandler)
    if len(sys.argv) == 1: 
       main_automation()
    else:
       main_cli_args()
