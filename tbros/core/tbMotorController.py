#!/usr/bin/env python
import platform
import atexit

#if platform.system() is not 'Darwin':

#Very Pi imports
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor
#from tbAnalogSensors.py import SPIAnalog
import time as Time
import atexit
import threading
import random as Random
import Queue
import sys
import time
from subprocess import call
import subprocess
import serial
import datetime
import logging
import re
import signal

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
            logger.info("Driving Forward Motor %s " , mId)
            self.motor[mId].run(Adafruit_MotorHAT.FORWARD)
        elif direction == "REVERSE":
            logger.info("Driving Backward Motor %s", mId)
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
    def turnOffMotor(self, mId):
        print "Releasing motor %s" % mId
        self.motor[mId].run(Adafruit_MotorHAT.RELEASE)

    def cleanClose():
        self.stopMotors()

#A thread that reads encoder data from Arduino over Serial
class tBEncoderCapture(threading.Thread):

    def __init__(self, stopper, queue, devid = '/dev/ttyUSB0'):
        # type: (object, object, object) -> object

        threading.Thread.__init__(self)
        self.stopper = stopper
        self.q = queue

        logger.info("Trying to open Serial arduino: %s ", devid)
        try:
            self.Serial = serial.Serial(devid, 9600)
            pass
        except (IOError, ValueError):
            logger.error("Cannot open serial device: %s", devid)
            raise IOError

    def run(self):
        while not self.stopper.is_set():
            line = self.Serial.readline()
            res= re.findall("[-+]?\d+[\.]?\d*", line)
            try:
                if len(res) > 1:
                    enc1_vel = int(res[0])
                    enc2_vel = int(res[1])
                    self.q.put((enc1_vel, enc2_vel))
                else:
                    continue
            except ValueError:
                logger.error("Value error exception parsing serial data")
                pass
        logger.info("Exiting encoder thread now...")

#Implementation of Two wheeled robot assuming LEFT motor is 1 and RIGHT Motor is 3
class TwoWheelRobot():

    def __init__(self,stopper, queue, mode = "DIRECT"):

        self.LEFT_MOTOR = 1
        self.RIGHT_MOTOR = 3
        self.Directions = ["FORWARD", "REVERSE"]
        self.Turns = ["CLOCKWISE", "COUNTERCLOCKWISE"]
        self.tBMotors = MotorHatDCMotorController([self.LEFT_MOTOR, self.RIGHT_MOTOR], 0x60)
        self.mode = mode
        self.stopper = stopper
        self.q = queue

        if not (self.mode == 'DIRECT' or self.mode  == 'PID'):
            logger.error("Mode %s is not supported...", self.mode)
            exit()
        logger.info("Starting robot in %s mode", self.mode)

    def drive(self, direction, speed):

        try:
            self.Directions.index(direction)
        except ValueError:
            logger.error("Sorry, direction %s is not supported", direction)
            return

        if(direction==self.Directions[0]):
            self.tBMotors.setDirection(self.LEFT_MOTOR, "FORWARD")
            self.tBMotors.setDirection(self.RIGHT_MOTOR, "FORWARD")
        elif(direction==self.Directions[1]):
            self.tBMotors.setDirection(self.LEFT_MOTOR, "REVERSE")
            self.tBMotors.setDirection(self.RIGHT_MOTOR, "REVERSE")

        #Exit an active motor loop if running by calling stopper.set()
        self.stopper.set()
        time.sleep(0.05)
        self.stopper.clear()
        # Now that everything is set, run the two motors on a new thread
        driveThread = threading.Thread(group=None, target = self.linearmotioncontrol, name="current-motor-thread", args = [int(speed)])
        driveThread.start()
    #Turns the robot
    def turn(self, direction, angvel):

        try:
            self.Turns.index(direction)
        except ValueError:
            logger.error("Sorry, direction %s is not supported", direction)
            return

        if (direction == self.Turns[0]):
            self.tBMotors.setDirection(self.LEFT_MOTOR, "REVERSE")
            self.tBMotors.setDirection(self.RIGHT_MOTOR, "FORWARD")
        elif (direction == self.Turns[1]):
            self.tBMotors.setDirection(self.LEFT_MOTOR, "FORWARD")
            self.tBMotors.setDirection(self.RIGHT_MOTOR, "REVERSE")

        #self.Cleanup()
        #self.Stop()

        # Now that everything is set, run the two motors on a new thread
        driveThread = threading.Thread(group=None, target=self.angularmotioncontrol, name="current-motor-thread", args=[int(speed)])
        driveThread.join()
        driveThread.start()


    def GetSpeed(self):
        if isinstance(self.q, Queue.Queue) is True:
            return self.q.get()

    def linearmotioncontrol(self, target_speed):

        filename = "log_" + str(self.mode)+"_" + time.strftime("%H_%M_%S")+".txt"
        logfile = open(filename, "w")
        dt = 0.01
        integral_l = integral_r= 0
        Kp=0.1
        Ki=0.1
        while not self.stopper.is_set():
            current_speed = self.GetSpeed()
            error_l = target_speed - current_speed[0]
            error_r = target_speed - current_speed[1]
            pwm_l = (Kp*error_l) + (Ki * integral_l)
            pwm_r = (Kp*error_r) + (Ki * integral_r)
            logger.info("target: %s - current_left: %s - current_right: %s", str(target_speed), str(current_speed[0]), str(current_speed[1]))

            if (pwm_l > 100):
                pwm_l = 100
            elif (pwm_l < -100):
                pwm_l = -100
            else:
                integral_l = integral_l + (error_l*dt)

            if (pwm_r > 100):
                pwm_r = 100
            elif (pwm_r < -100):
                pwm_r = -100
            else:
                integral_r = integral_r + (error_r*dt)
            logstring = str(Kp)+"\t"+str(Ki)+"\t"+str(target_speed) + "\t" + \
            str(current_speed[0]) + "\t" + str(current_speed[1])
            
            logfile.write(logstring)
            pwm_l = self.scale(pwm_l)
            pwm_r = self.scale(pwm_r)
            self.tBMotors.runMotor(self.LEFT_MOTOR, int(pwm_l))
            self.tBMotors.runMotor(self.RIGHT_MOTOR, int(pwm_r))
            time.sleep(dt)

        logger.info("Exiting the DIRECT loop")
        logfile.close()

    def scale(self, value):
        return ((255-0)*(value-(-100))/(100-(-100)) + 0)

    def angularmotioncontrol(self, target_speed):
        return (100)
                
    def Stop(self):
        logger.info("Stopping Motors")
        self.tBMotors.stopMotors()

    def Cleanup(self):
        self.stopper.set()

def main():
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

    Q = Queue.Queue()
    #Initialize the stopper
    Stopper = threading.Event() 
    Stopper.clear()
    threads = []

    #try:
    feedbackProvider = tBEncoderCapture( Stopper, Q)
    myrobot = TwoWheelRobot(Stopper, Q,  mode = "DIRECT")
    feedbackProvider.start()
    threads.append(feedbackProvider)
    threads.append(myrobot)

    handler = SignalHandler(Stopper, threads)
    handler.SetRobot(myrobot)
    signal.signal(signal.SIGINT, handler)

    #Direct drive Tests: Power bot at max PWM for 10 seconds and record velocity
    speeds = [200, 500, 1200]
    for i in speeds:
        logger.info("Driving forward at %d", i)
        myrobot.drive("FORWARD", int(i))
        time.sleep(10)
        myrobot.Cleanup()
    myrobot.Stop()

    #speeds = [0, 50, 100, 150, 200, 255]
    #for i in speeds:
    #    logger.info("Driving forward at %d", int(i))
    #    myrobot.turn("LEFT", int(i))
    #    time.sleep(5)
    #logger.info("Look I got here!")
    myrobot.Cleanup()
    myrobot.Stop()

def exithandler():
    myrobot.Cleanup()
    myrobot.Stop() 

if __name__ == '__main__':
    atexit.register(exithandler)
    main()
