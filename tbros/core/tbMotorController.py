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
        self.Directions = ["FORWARD", "REVERSE", "LEFT", "RIGHT"]
        self.tBMotors = MotorHatDCMotorController([self.LEFT_MOTOR, self.RIGHT_MOTOR], 0x60)
        self.mode = mode
        self.stopper = stopper
        self.q = queue
        #if colavoid is True:
        #    self.analogsensors = SPIAnalog()
        if not (self.mode is ('DIRECT' or 'PID')):
            logger.error("Mode %s is not supported...", self.mode)
            exit()
        logger.info("Starting robot in %s mode", self.mode)

    def Drive(self,direction, speed):

        try:
            self.Directions.index(direction)
        except ValueError:
            logger.error("Sorry, direction %s is not supported", direction)
            return

        #self.Stop()

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

        # Now that everything is set, run the two motors on a new thread
        driveThread = threading.Thread(group=None, target = self.RunMotor, name="current-motor-thread", args = [int(speed)])
        driveThread.start()
        #driveThread.join()

    def GetSpeed(self):

        if isinstance(self.q, Queue.Queue) is True:
            #logger.info("Wheel speed is A:%d \\t B:%d", self.q.get()[0], self.q.get()[1])
            return self.q.get()

    def RunMotor(self,speed):

	dT = 0.01
        if str(self.mode) == "PID":
            filename = "log_" + str(self.mode)+"_" + time.strftime("%H_%M_%S")+".txt"
            File = open(filename, "w")
            Kp1 = 1
            Kd1 = 0
            Ki1 = 0
            error_prior = 0
            integral=0

            while not self.stopper.is_set():
                startT = time.time()
                wheel_speeds = self.GetSpeed();
                error = speed - (abs(wheel_speeds[0])+abs(wheel_speeds[1]))/2
                integral = integral + (error*dT)
                derivative = (error - error_prior)/dT
                output = Kp1 * error + Ki1 * integral + Kp1 * derivative
                #TODO Implement mapping between output and PWM
                self.tBMotors.runMotor(self.LEFT_MOTOR, output)
                self.tBMotors.runMotor(self.RIGHT_MOTOR, output)
                time.sleep(dT)
                #dT = time.time() - startT
                logstring = str(str(dT) + "\t" + str(wheel_speeds[0]) + "\t" + str(wheel_speeds[1]) + "\t" + str(error) + "\t" + str(integral) + "\t" + str(derivative) + "\t" + str(output) +"\n")
                File.write(logstring)
            logger.info("Exiting the PID loop")
            File.close()

        elif str(self.mode) == "DIRECT":
            filename = "log_" + str(self.mode)+"_" + time.strftime("%H_%M_%S")+".txt"
            File = open(filename, "w")
            pwm = 50
            while not self.stopper.is_set():
                wheel_speeds = self.GetSpeed()
                error = speed - (abs(wheel_speeds[0])+abs(wheel_speeds[1]))/2
                if error < 0:
		    continue
                else:
                    pwm = pwm + 1
                self.tBMotors.runMotor(self.LEFT_MOTOR, pwm)
                self.tBMotors.runMotor(self.RIGHT_MOTOR, pwm)
                logstring = str(str(pwm) + "\\t" + str(speed) + "\t" + str(dT) + "\\t" + str(wheel_speeds[0]) + "\\t" + str(wheel_speeds[1]) +"\\n")
                File.write(logstring)
            logger.info("Exiting the DIRECT loop")
            File.close()

    def Stop(self):
        logger.info("Stopping Motors")
        self.tBMotors.stopMotors()
        #with lock:
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
    #except (IOError, ValueError, TypeError):
    #    logger.error("Something wrong")
    #    exit()

    #Direct Drive Tests: Power bot at max PWM for 10 seconds and record velocity
    dutycycles = [0, 50, 100, 150, 200, 255]
    #flsor i in dutycycles:
    logger.info("Driving forward at %d",int(sys.argv[1]))
    myrobot.Drive("FORWARD", int(sys.argv[1]))
    time.sleep(5)
    logger.info("Look I got here!")
    myrobot.Cleanup()
    myrobot.Stop()

if __name__ == '__main__':
    main()
