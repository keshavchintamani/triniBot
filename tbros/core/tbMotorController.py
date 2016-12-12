#!/usr/bin/env python
import platform
import atexit

#if platform.system() is not 'Darwin':

#Very Pi imports
#from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor
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

        # Now that everything is set, run the two motors on a new thread
        driveThread = threading.Thread(group=None, target=self.angularmotioncontrol, name="current-motor-thread", args=[int(speed)])
        driveThread.start()


    def GetSpeed(self):
        if isinstance(self.q, Queue.Queue) is True:
            return self.q.get()

    def linearmotioncontrol(self, target_speed):

        dT = 0.01
        if str(self.mode) == "PID":
            filename = "log_" + str(self.mode)+"_" + time.strftime("%H_%M_%S")+".txt"
            logfile = open(filename, "w")
            Kp = 0.1
            Kd = 0.1
            Ki = 0.1
            error_prior = 0
            integral = 0
            while not self.stopper.is_set():
                startT = time.time()
                current_speed = self.GetSpeed()
                error = target_speed - (abs(current_speed[0]) + abs(current_speed[1])) / 2
                integral = integral + (error*dT)
                derivative = (error - error_prior)/dT
                output = Kp * error + Ki * integral + Kp * derivative
                #TODO Implement mapping between output and PWM
                self.tBMotors.runMotor(self.LEFT_MOTOR, output)
                self.tBMotors.runMotor(self.RIGHT_MOTOR, output)
                #dT = time.time() - startT
                time.sleep(dT)
                error_prior = error
                logstring = str(dT) + "\t" + str(current_speed[0]) + "\t" + str(current_speed[1]) + "\t" + str(error) + "\t" + str(integral) + "\t" + str(derivative) + "\t" + str(output) +"\n"
                logfile.write(logstring)
            logger.info("Exiting the PID loop")
            logfile.close()

        elif str(self.mode) == "DIRECT":
            filename = "log_" + str(self.mode)+"_" + time.strftime("%H_%M_%S")+".txt"
            logfile = open(filename, "w")
            w1_pwm = 50
            w2_pwm = 50
            threshold = 0.02 #Acceptable error threshold
            while not self.stopper.is_set():
                current_speed = self.GetSpeed()
                w1_error = (target_speed - abs(current_speed[0]))/target_speed
                w2_error = (target_speed - abs(current_speed[1]))/target_speed
                if not (w1_error <=threshold or w1_error >= -threshold):
                    w1_pwm = w1_pwm + 1
                    self.tBMotors.runMotor(self.LEFT_MOTOR, w1_pwm)
                if not (w2_error <= threshold or w2_error >= -threshold):
                    w2_pwm = w2_pwm + 1
                    self.tBMotors.runMotor(self.RIGHT_MOTOR, w2_pwm)
                logstring = str(target_speed) + "\t" + str(current_speed[0]) + "\t" + str(current_speed[1]) + "\n"
                logfile.write(logstring)
            logger.info("Exiting the DIRECT loop")
            logfile.close()

    def angularmotioncontrol(self, target_speed):

        dT = 0.01
        if str(self.mode) == "PID":
            filename = "log_" + str(self.mode) + "_" + time.strftime("%H_%M_%S") + ".txt"
            logfile = open(filename, "w")
            Kp = 1
            Kd = 0
            Ki = 0
            error_prior = 0
            integral = 0

            while not self.stopper.is_set():
                startT = time.time()
                current_speed = self.GetSpeed()
                error = target_speed - (abs(current_speed[0]) + abs(current_speed[1])) / 2
                integral = integral + (error * dT)
                derivative = (error - error_prior) / dT
                output = Kp * error + Ki * integral + Kp * derivative
                # TODO Implement mapping between output and PWM
                self.tBMotors.runMotor(self.LEFT_MOTOR, output)
                self.tBMotors.runMotor(self.RIGHT_MOTOR, output)
                # dT = time.time() - startT
                error_prior = error
                time.sleep(dT)
                logstring = str(dT) + "\t" + str(current_speed[0]) + "\t" + str(current_speed[1]) + "\t" + str(
                    error) + "\t" + str(integral) + "\t" + str(derivative) + "\t" + str(output) + "\n"
                logfile.write(logstring)
            logger.info("Exiting the PID loop")
            logfile.close()

        elif str(self.mode) == "DIRECT":
            filename = "log_" + str(self.mode) + "_" + time.strftime("%H_%M_%S") + ".txt"
            logfile = open(filename, "w")
            w1_pwm = 50
            w2_pwm = 50
            # Continue from here: Independent wheel speed control
            threshold = 0.05  # Tolerable error threshold
            while not self.stopper.is_set():
                current_speed = self.GetSpeed()
                # Calculate the error percentage
                error = (target_speed - (abs(current_speed[0])+abs(current_speed[1]))/2)/ target_speed
                # TODO Find an isinrange() function
                if not error >= 0 or error <= threshold:
                    w1_pwm = w1_pwm + 1
                    w2_pwm = w2_pwm + 1
                    self.tBMotors.runMotor(self.LEFT_MOTOR, w1_pwm)
                    self.tBMotors.runMotor(self.RIGHT_MOTOR, w2_pwm)
                logstring = str(target_speed) + "\t" + str(dT) + "\\t" + str(current_speed[0]) + "\\t" + str(current_speed[1]) + "\\n"
                logfile.write(logstring)
            logger.info("Exiting the DIRECT loop")
            logfile.close()

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
    speeds = [0, 50, 100, 150, 200, 255]
    for i in speeds:
        logger.info("Driving forward at %d",int(sys.argv[1]))
        myrobot.drive("FORWARD", int(sys.argv[1]))
        time.sleep(5)

    speeds = [0, 50, 100, 150, 200, 255]
    for i in speeds:
        logger.info("Driving forward at %d", int(sys.argv[1]))
        myrobot.turn("LEFT", int(sys.argv[1]))
        time.sleep(5)
    logger.info("Look I got here!")
    myrobot.Cleanup()
    myrobot.Stop()

if __name__ == '__main__':
    main()
