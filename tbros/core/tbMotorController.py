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
    
    #Send an r to the arduino to reset the odo
    def reset(self):
        time.sleep(0.01)
        logger.info("Resetting the encoder counter...")
        self.Serial.write('r')

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

#Documentation
#TrackedTriniBot is a specialized tracked platform using the 150:1 micrometal gearmotors
#It assumes certain dimensions of the wheels for its odometry
#Changes to the wheels or transmision should be reflected in the parameters
class TrackedTrinibot():

    def __init__(self,stopper, queue, encoderthread, mode ="pid"):

        self.mode = mode
        self.encoderThread = encoderthread
        self.LEFT_MOTOR = 1
        self.RIGHT_MOTOR = 3
        self.Directions = ["FORWARD", "REVERSE"]
        self.Turns = ["CLOCKWISE", "COUNTERCLOCKWISE"]
        self.tBMotors = MotorHatDCMotorController([self.LEFT_MOTOR, self.RIGHT_MOTOR], 0x60)
        self.stopper = stopper
        self.q = queue
        self.isRunning = False
        #Give Kp and Ki some initial values 
        self.Ki = 0.1
        self.Kp = 0.1

    def drive(self, direction, target_speed, target_dist = 20000):
        self.Stop()
        try:
            self.Directions.index(direction)
        except ValueError:
            logger.error("Sorry, direction %s is not supported", direction)
        if(direction==self.Directions[0]):
            self.tBMotors.setDirection(self.LEFT_MOTOR, "FORWARD")
            self.tBMotors.setDirection(self.RIGHT_MOTOR, "FORWARD")
        elif(direction==self.Directions[1]):
            self.tBMotors.setDirection(self.LEFT_MOTOR, "REVERSE")
            self.tBMotors.setDirection(self.RIGHT_MOTOR, "REVERSE")
        if(self.isRunning == True):
            logger.info("Motor was running so stopping it by changing flag..")
            self.isRunning = False
            time.sleep(0.01)
        self.isRunning = True
        self.speedcontrol(target_speed, target_dist)
        # Now that everything is set, run the two motors on a new thread
        #driveThread = threading.Thread(group=None, target=self.speedcontrol, name="current-motor-thread", args=[int(target_speed),float(target_dist) ])
        #driveThread.start()

    def setKp(self, kp):
            self.Kp = float(kp)
    def setKi(self, ki):
            self.Ki = float(ki)

    def speedcontrol(self, target_speed, target_distance=200000):

        filename = "log_" + time.strftime("%Hss_%M_%S")+".txt"
        logfile = open(filename, "w")
        dt = 0.01
        integral_l = integral_r= 0
        #Kp=0.1
        #Ki=0.1
        dist_travelled =last_dist_travelled= 0
        old_pwm_r = old_pwm_l = 0
        pulse_revolution=1800
        cm_pulse=0.006806
        deg_pulse=0.2
        last_speed_l=last_speed_r=0
        #Reset the counter through the encoder thread
        self.encoderThread.reset()
        #TODO Update to read the second encoder
        if self.mode == "pid":
            logger.info("mode=%s Kp = %f - Ki = %f", self.mode, self.Kp, self.Ki) 
            while self.isRunning == True and not self.stopper.is_set():
                current_odo = self.GetOdo()
                #Degrees travelled in centimeteresi since last pass
                dist_travelled = int(current_odo[1])
                odo = abs(dist_travelled)*cm_pulse
                current_speed = 0.2*(abs(dist_travelled - last_dist_travelled))/dt
                logger.info("odo: %f\t deg/s:%f", odo, current_speed)
                if odo >= target_distance:
                        self.isRunning = False
                error_l = target_speed - current_speed
                error_r = target_speed - current_speed
                pwm_l = (self.Kp*error_l) + (self.Ki * integral_l)
                pwm_r = (self.Kp*error_r) + (self.Ki * integral_r)

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
                
                
                logstring = str(self.Kp)+"\t"+str(self.Ki)+"\t"+str(target_speed) + "\t" + \
                    str(current_odo) + "\t" + str(current_speed) + "\t" + \
                    str(target_distance) + "\t" + str(odo) + "\n"

                logfile.write(logstring)
                pwm_l = self.scale(pwm_l)
                pwm_r = self.scale(pwm_r)

                if(not pwm_l == old_pwm_l): 
                    self.tBMotors.runMotor(self.LEFT_MOTOR, int(pwm_l))
                if (not pwm_r == old_pwm_r):
                    self.tBMotors.runMotor(self.RIGHT_MOTOR, int(pwm_r))
                last_dist_travelled = dist_travelled
                old_pwm_l = pwm_l
                old_pwm_r = pwm_r
                last_speed_l = float(current_speed)
                last_speed_r = float(current_speed)
                time.sleep(dt)
        else:
            self.tBMotors.runMotor(self.LEFT_MOTOR, target_speed)
            self.tBMotors.runMotor(self.RIGHT_MOTOR, target_speed)
                #logger.info("New command received... exiting the control loop")
        self.Stop()
        logger.info("target:%fM - desired:%fM", float(odo), float(target_distance))
        logfile.close()

    def GetOdo(self):
        if isinstance(self.q, Queue.Queue) is True:
            return self.q.get()

    def scale(self, value):
        return ((255-0)*(value-(-100))/(100-(-100)) + 0)

    def Stop(self):
        logger.info("Stopping Motors")
        self.isRunning = False
        time.sleep(0.01)
        self.tBMotors.stopMotors()

    def Cleanup(self):
        self.stopper.set()

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

    Q = Queue.Queue()
    #Initialize the stopper
    Stopper = threading.Event()
    Stopper.clear()
    threads = []

    #try:
    feedbackProvider = tBEncoderCapture( Stopper, Q)
    myrobot = TrackedTrinibot(Stopper, Q, feedbackProvider)
    feedbackProvider.start()
    threads.append(feedbackProvider)
    threads.append(myrobot)

    handler = SignalHandler(Stopper, threads)
    handler.SetRobot(myrobot)
    signal.signal(signal.SIGINT, handler)

    #Direct drive Tests: Power bot at max PWM for 10 seconds and record velocity
    #for i in speeds:
    KP=[0.1, 0.2, 0.4, 0.8]#0.5, 0.6, 0.7, 0.8, 0.9]
    KI=[0.1, 0.2, 0.4, 0.8]# 0.5, 0.6, 0.7, 0.8, 0.9]
    for Kp in KP:
        for Ki in KI:
            myrobot.setKi(Kp)
            myrobot.setKp(Ki)
            myrobot.drive("FORWARD", 100, 5)
            time.sleep(5)

            
        logger.info("Invalid number of arguments")
    #myrobot.Stop()

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

    Q = Queue.Queue()
    #Initialize the stopper
    Stopper = threading.Event()
    Stopper.clear()
    threads = []

    feedbackProvider = tBEncoderCapture( Stopper, Q)
    myrobot = TrackedTrinibot(Stopper, Q, feedbackProvider)
    feedbackProvider.start()
    threads.append(feedbackProvider)
    threads.append(myrobot)

    handler = SignalHandler(Stopper, threads)
    handler.SetRobot(myrobot)
    signal.signal(signal.SIGINT, handler)

    myrobot.setKi(0.1)
    myrobot.setKp(0.08)
    if len(sys.argv) == 2:
        myrobot.drive("FORWARD", int(sys.argv[1])) 
    elif len(sys.argv)==3:
        myrobot.drive("FORWARD", int(sys.argv[1]), int(sys.argv[2]))
    else:
        logger.info("Invalid number of arguments")




def exithandler():
    #global myrobot
    #myrobot.Cleanup()
    #myrobot.Stop()
    print "Goodbye!"
if __name__ == '__main__':
    atexit.register(exithandler)
    if len(sys.argv) == 1: 
       main_automation()
    else:
       main_cli_args()
