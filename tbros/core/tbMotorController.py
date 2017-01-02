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

class tBSerialReader():

    def __init__(self, devid):

        self.devid = devid
        self.startserial()

    def _startserial(self):

        logger.info("Trying to open Serial arduino: %s ", self.devid)
        try:
            self.Serial = serial.Serial(self.devid, 9600)
            pass
        except (IOError, ValueError):
            logger.error("Cannot open serial device: %s", delf.devid)
            raise IOError

    def resetserial(self):
        self.Serial.write('r')
        # wait for acknowledgement
        while (not self.Serial.read() == 'd'):
            logger.info("Waiting for confirmation")
        self.q.empty()
        logger.info("Encoder counter reset...")

    def readserial(self):

        line = self.Serial.readline()
        res = re.findall("[-+]?\d+[\.]?\d*", line)
        try:
            if len(res) > 1:
                enc1_vel = int(res[0])
                enc2_vel = int(res[1])
                return ((enc1_vel, enc2_vel))
        except ValueError:
            logger.error("Value error exception parsing serial data")
            pass

#Documentation
#TrackedTriniBot is a specialized tracked platform using the 150:1 micrometal gearmotors
#It assumes certain dimensions of the wheels for its odometry
#Changes to the wheels or transmision should be reflected in the parameters
class TrackedTrinibot():

    def __init__(self,stopper, encoder_dev = '/dev/ttyUSB0', motorcontrol_dev = 0x60):

        self.serial = tBSerialReader(encoder_dev)
        self.LEFT_MOTOR = 1
        self.RIGHT_MOTOR = 3
        self.Directions = ["FORWARD", "REVERSE", "LEFT", "RIGHT"]
        self.tBMotors = MotorHatDCMotorController([self.LEFT_MOTOR, self.RIGHT_MOTOR], motorcontrol_dev)
        self.stopper = stopper
        self.isRunning = False
        self.Ki = 0.1
        self.Kp = 0.01
        self.logfileindex=0
    
    def set_direction(self, direction):
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
        elif (direction == self.Directions[0]):
            self.tBMotors.setDirection(self.LEFT_MOTOR, "FORWARD")
            self.tBMotors.setDirection(self.RIGHT_MOTOR, "REVERSE")
        elif (direction == self.Directions[1]):
            self.tBMotors.setDirection(self.LEFT_MOTOR, "REVERSE")
            self.tBMotors.setDirection(self.RIGHT_MOTOR, "FORWARD")

    def openlogger(self):
        filename = "log_" + time.strftime("%Hss_%M_%S_") + str(self.logfileindex) + ".dat"
        return(open(filename, "w"))

    def drive_at_speed(self, direction, target_speed=300):
        self.Stop()
        self.set_direction(direction)
        if(self.isRunning == True):
            logger.info("Motor was running so stopping it by changing flag..")
            time.sleep(0.01)
            return (False)
        self.isRunning = True
        logfile = self.openlogger()
        dt = 0.01
        integral_l = integral_r= 0
        last_odo= 0
        odo = 0
        Odo = self.serial.readserial()
        start_odo = int(Odo[1])
        # track travel in centimeters/encoder pulse is 0.006806 for 150:1 and 0.0102 for 100:1
        cm_pulse= 0.0102 #0.006806
        # degrees/encoder pulse is 0.2 for 150:1 and 0.3 for 100:1 - which one is it!!!???
        deg_pulse=0.2

        #TODO Update to read the second encoder
        #TODO Add Differentiator to PI controller
        while self.isRunning == True and not self.stopper.is_set():
            current_odo = self.serial.readserial()
            odo =  abs(current_odo[1]-start_odo)
            current_speed = deg_pulse*(current_odo[1] - last_odo)/dt
            error_l = target_speed - current_speed
            error_r = target_speed - current_speed
            error_percent = abs(error_l)/target_speed*100
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
            pwm_l = self.scalepwm(pwm_l)
            pwm_r = self.scalepwm(pwm_r)
            self.tBMotors.runMotor(self.LEFT_MOTOR, int(pwm_l))
            self.tBMotors.runMotor(self.RIGHT_MOTOR, int(pwm_r))
            last_odo = current_odo[1]
            logstring = str(self.Kp) + "\t" + str(self.Ki) + "\t" + str(target_speed) + "\t" + \
                        str(currrent_speed) + "\t" + str(odo * cm_pulse) + "\t" + \
                        str(error_percent) + "\n"
            logfile.write(logstring)
            if error_percent < 2:
                self.isRunning = False
            time.sleep(dt)

        self.Stop()
        logger.info("target:%fM - desired:%fM", float(odo*cm_pulse), float(target_distance))
        logfile.close()
        self.logfileindex= self.logfileindex + 1
        return (True)

    def drive_to_distance(self, direction, target_distance=10):

        self.set_direction(direction)
        if(self.isRunning == True):
            #self.isRunning = False
            return(False)
            time.sleep(0.01)
        self.isRunning = True

        logfile = self.openlogger()
        dt = 0.01
        integral_l = integral_r= 0
        last_odo= 0
        odo = 0
        Odo=self.serial.readserial()
        start_odo = int(Odo[1])
        # track travel in centimeters/encoder pulse is 0.006806 for 150:1 and 0.0102 for 100:1
        cm_pulse= 0.006806
        # degrees/encoder pulse is 0.2 for 150:1 and 0.3 for 100:1 - which one is it!!!???
        deg_pulse=0.2

        #TODO Update to read the second encoder
        while self.isRunning == True and not self.stopper.is_set():
            current_odo = self.serial.readserial()
            odo =  abs(current_odo[1]-start_odo)
            current_speed = deg_pulse*(current_odo[1] - last_odo)/dt
            error_l = target_distance - odo*cm_pulse
            error_r = error_l
            error_percent = abs(error_l)/target_distance*100
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

            pwm_l = self.scalepwm(pwm_l)
            pwm_r = self.scalepwm(pwm_r)

            self.tBMotors.runMotor(self.LEFT_MOTOR, int(pwm_l))
            self.tBMotors.runMotor(self.RIGHT_MOTOR, int(pwm_r))
            last_odo = current_odo[1]
            if error_percent < 2:
                self.isRunning = False
            logstring = str(self.Kp)+"\t"+str(self.Ki)+"\t" + \
                        str(target_distance) + "\t" + str(odo*cm_pulse) + "\t" + \
                        str(error_percent) + "\t" + str(pwm_l) +"\t" + \
                        str(pwm_r) +  "\n"

            logfile.write(logstring)
            time.sleep(dt)

        self.Stop()
        logger.info("target:%fM - desired:%fM", float(odo*cm_pulse), float(target_distance))
        logfile.close()
        self.logfileindex = self.logfileindex + 1
        return(True)

    def setKp(self, kp):
            self.Kp = float(kp)
    def setKi(self, ki):
            self.Ki = float(ki)

    #Scales PWM from -100 to +100 to 0 to 255 
    def scalepwm(self, value):
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

    #Initialize the stopper
    Stopper = threading.Event()
    Stopper.clear()
    threads = []

    myrobot = TrackedTrinibot(Stopper, "/dev/ttyUSB0")
    threads.append(myrobot)

    handler = SignalHandler(Stopper, threads)
    handler.SetRobot(myrobot)
    signal.signal(signal.SIGINT, handler)

    KP=[0.01, 0.05, 0.1, 0.2]#0.5, 0.6, 0.7, 0.8, 0.9]
    KI=[0.01, 0.05, 0.1, 0.2]# 0.5, 0.6, 0.7, 0.8, 0.9]
    for Kp in KP:
        for Ki in KI:
            logger.info("Running motor at Kp: %f and Ki: %f", Kp, Ki)  
            myrobot.setKi(Kp)
            myrobot.setKp(Ki)
            status = myrobot.drive_to_distance("FORWARD", 5)
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

    myrobot = TrackedTrinibot(Stopper, "/dev/ttyUSB0")
    threads.append(myrobot)

    handler = SignalHandler(Stopper, threads)
    handler.SetRobot(myrobot)
    signal.signal(signal.SIGINT, handler)

    myrobot.setKi(0.05)
    myrobot.setKp(0.01)
    if sys.argv[1] == 'd':
        myrobot.drive_to_distance("FORWARD", int(sys.argv[2])) 
    elif sys.argv[1] == 's':
        myrobot.drive_at_speed("FORWARD", int(sys.argv[2]))
    else:
        logger.info("Invalid number of arguments")
   
    myrobot.Cleanup()


def exithandler():

    logger.info("Goodbye!")

if __name__ == '__main__':
    atexit.register(exithandler)
    if len(sys.argv) == 1: 
       main_automation()
    else:
       main_cli_args()
