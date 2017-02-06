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
#import matplotlib
#matplotlib.use('Qt4Agg')
#import matplotlib.pyplot as plt
#import matplotlib.gridspec as gridspec
#import matplotlib.patches as mpatches

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
            self.Serial = serial.Serial(self.devid, 115200)
            time.sleep(1)
            pass
        except (IOError, ValueError):
            print("Cannot open serial device: %s", self.devid)
            raise IOError

    def resetserial(self):
        self.Serial.write('r')
        # wait for acknowledgement
        while (not self.Serial.read() == 'd'):
            print("Waiting for confirmation")

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



#Documentation
#TrackedTriniBot is a specialized tracked platform using the 150:1 micrometal gearmotors
#It assumes certain dimensions of the wheels for its odometry
#Changes to the wheels or transmision should be reflected in the parameters
class TrackedTrinibot():

    def __init__(self,stopper, tacho_cb, encoder_dev = '/dev/ttyACM0', motorcontrol_dev = 0x60):

        self.LEFT_MOTOR = 1
        self.RIGHT_MOTOR = 3
        self.Directions = ["FORWARD", "REVERSE", "LEFT", "RIGHT"]
        self.tBMotors = MotorHatDCMotorController([self.LEFT_MOTOR, self.RIGHT_MOTOR], motorcontrol_dev)
        self.stopper = stopper
        self.isRunning = False
        self.Ki = 0.0
        self.Kp = 0.01
        self.Kd = 0.01
        self.logfileindex=0
        self.serial = tBSerialReader(encoder_dev)
        self.tach_callback = tacho_cb
    def set_direction(self, direction):
        try:
            self.Directions.index(direction)
        except ValueError:
            logger.error("Sorry, direction %s is not supported", direction)
        
        if(direction==self.Directions[0]):
            self.tBMotors.setDirection(self.LEFT_MOTOR, "FORWARD")
            self.tBMotors.setDirection(self.RIGHT_MOTOR, "REVERSE")
        elif(direction==self.Directions[1]):
            self.tBMotors.setDirection(self.LEFT_MOTOR, "REVERSE")
            self.tBMotors.setDirection(self.RIGHT_MOTOR, "FORWARD")
        elif (direction == self.Directions[2]):
            self.tBMotors.setDirection(self.LEFT_MOTOR, "REVERSE")
            self.tBMotors.setDirection(self.RIGHT_MOTOR, "REVERSE")
        elif (direction == self.Directions[3]):
            self.tBMotors.setDirection(self.LEFT_MOTOR, "FORWARD")
            self.tBMotors.setDirection(self.RIGHT_MOTOR, "FORWARD")

    def openlogger(self, path="logs/"):
    #    self.logfilepath = path + "log_" + str(self.logfileindex) + ".dat"
         return(open("logs/" + "log_" + str(self.logfileindex) + ".dat", "w")
)

    def drive_at_speed(self, target_speed=300):
        if(self.isRunning == True):
            return(False)
            time.sleep(0.01)
        self.isRunning = True
        dt = 0.1

        integral_l = integral_r= 0
        error_r = error_l = error_last_l = error_last_r = 0
        derivative_length=64
        derivate_counter=0
        MAX_PWM= 100# Percent duty cycle
        current_odo = [0, 0]
        circumference = 188.495592
        logger.info("Kp: %0.3f\tKi: %0.3f\t Kd: %0.3f", self.Kp, self.Ki, self.Kd)
        odo = 0
        duty = 255
        if target_speed> 0 :
            self.set_direction("FORWARD")
            direction = -1 
        if target_speed < 0 :
            self.set_direction("REVERSE")
            direction = 1
        to_drive_l =  -direction*(abs(target_speed))*5 
        to_drive_r =  direction*(abs(target_speed))*5 
         
        old_odo_r = old_odo_l = 0
        while self.isRunning == True and not self.stopper.is_set():
            pwm_r = (self.Kp*error_r) + (self.Ki * integral_r) + (self.Kd * integral_r)
            pwm_l = 0#(self.Kp*error_l) + (self.Ki * integral_l) + (self.Kd * integral_l)
            if (pwm_l > MAX_PWM):
                pwm_l = MAX_PWM
            elif (pwm_l < -MAX_PWM):
                pwm_l = -MAX_PWM
            else:
                integral_l = integral_l + (error_l*dt)
            derivative_l = (error_l - error_last_l) / dt;

            if (pwm_r > MAX_PWM):
                pwm_r =  MAX_PWM
            elif (pwm_r < -MAX_PWM):
                pwm_r = -MAX_PWM
            else:
                integral_r = integral_r + (error_r*dt)
            derivative_r = (error_r - error_last_r) / dt;
 
            if(pwm_r > 0 ):
                self.tBMotors.setDirection(self.RIGHT_MOTOR, "FORWARD")
            if pwm_r < 0 :
                self.tBMotors.setDirection(self.RIGHT_MOTOR, "REVERSE")
            pwm_r = self.scalepwm(abs(pwm_r), 0, MAX_PWM, duty)
            
            if(pwm_l > 0 ):
                self.tBMotors.setDirection(self.LEFT_MOTOR, "FORWARD")
            if pwm_l < 0 :
                self.tBMotors.setDirection(self.LEFT_MOTOR, "REVERSE")
            pwm_l = self.scalepwm(abs(pwm_l), 0, MAX_PWM, duty)

            if derivate_counter == 0:
                pwm_l=pwm_r=50 
            #self.tBMotors.runMotor(self.RIGHT_MOTOR, int(pwm_r))
            #self.tBMotors.runMotor(self.LEFT_MOTOR, int(pwm_l))
            current_odo = self.serial.readserial()
            #First pass 
            if derivate_counter == 0:
                logger.info("in derivate_counter")
                start_odo = current_odo[1]
                # = to_drive_l + current_odo[0]
            odo = current_odo[1]-start_odo

            s = (odo - old_odo_r)/dt 
            error_r = to_drive_r - 0 
            error_l = to_drive_l - (old_odo_l-current_odo[0])/dt
            
            print "{}".format(s)
            #error_pr = 100*abs(error_r/to_drive_r)
            #error_pl = 100*abs(error_l/to_drive_l)
            #if (error_pr < 0.5 or error_pl < 0.5):
            #    self.tBMotors.stopMotor(self.RIGHT_MOTOR)
            #    self.tBMotors.stopMotor(self.LEFT_MOTOR) 
            #    self.isRunning = False

            derivate_counter = derivate_counter + 1
            if (derivate_counter % derivative_length == 0):
                error_last_r = error_r
                error_last_l = error_l
            old_odo_r = odo
            old_odo_l = current_odo[0]
            time.sleep(dt)

        logger.info("counts %d", derivate_counter)
        logger.info("error l:%f r:%f", error_pl, error_pr)
        self.tBMotors.stopMotors()
        return(True)

       

    def drive_to_distance(self, target_distance=188.49):
       
        if(self.isRunning == True):
            return(False)
            time.sleep(0.01)
        self.isRunning = True
        dt = 0.009

        integral_l = integral_r= 0
        error_r = error_l = error_last_l = error_last_r = 0
        derivative_length=64
        derivate_counter=0
        MAX_PWM= 100# Percent duty cycle
        current_odo = [0, 0]
        circumference = 188.495592
        logger.info("Kp: %0.3f\tKi: %0.3f\t Kd: %0.3f", self.Kp, self.Ki, self.Kd)
        odo = 0
        duty = 255
        if target_distance > 0 :
            self.set_direction("FORWARD")
            direction = -1 
        if target_distance < 0 :
            self.set_direction("REVERSE")
            direction = 1
        to_drive_l =  -direction*(abs(target_distance)/circumference)*1800 
        to_drive_r =  direction*(abs(target_distance)/circumference)*1800 
         
        while self.isRunning == True and not self.stopper.is_set():
            pwm_r = (self.Kp*error_r) + (self.Ki * integral_r) + (self.Kd * integral_r)
            pwm_l = (self.Kp*error_l) + (self.Ki * integral_l) + (self.Kd * integral_l)
            if (pwm_l > MAX_PWM):
                pwm_l = MAX_PWM
            elif (pwm_l < -MAX_PWM):
                pwm_l = -MAX_PWM
            else:
                integral_l = integral_l + (error_l*dt)
            derivative_l = (error_l - error_last_l) / dt;

            if (pwm_r > MAX_PWM):
                pwm_r =  MAX_PWM
            elif (pwm_r < -MAX_PWM):
                pwm_r = -MAX_PWM
            else:
                integral_r = integral_r + (error_r*dt)
            derivative_r = (error_r - error_last_r) / dt;
 
            if(pwm_r > 0 ):
                self.tBMotors.setDirection(self.RIGHT_MOTOR, "FORWARD")
            if pwm_r < 0 :
                self.tBMotors.setDirection(self.RIGHT_MOTOR, "REVERSE")
            pwm_r = self.scalepwm(abs(pwm_r), 0, MAX_PWM, duty)
            
            if(pwm_l > 0 ):
                self.tBMotors.setDirection(self.LEFT_MOTOR, "FORWARD")
            if pwm_l < 0 :
                self.tBMotors.setDirection(self.LEFT_MOTOR, "REVERSE")
            pwm_l = self.scalepwm(abs(pwm_l), 0, MAX_PWM, duty)

            if derivate_counter == 0:
                pwm_l=pwm_r=50 
            self.tBMotors.runMotor(self.RIGHT_MOTOR, int(pwm_r))
            self.tBMotors.runMotor(self.LEFT_MOTOR, int(pwm_l))
            current_odo = self.serial.readserial()
            #First pass 
            if derivate_counter == 0:
                logger.info("in derivate_counter")
                setpoint_r = to_drive_r + current_odo[1]
                setpoint_l = to_drive_l + current_odo[0]
            error_r = setpoint_r - current_odo[1]
            error_l = setpoint_l - current_odo[0]
            error_pr = 100*abs(error_r/to_drive_r)
            error_pl = 100*abs(error_l/to_drive_l)
            if (error_pr < 0.5 or error_pl < 0.5):
                self.tBMotors.stopMotor(self.RIGHT_MOTOR)
                self.tBMotors.stopMotor(self.LEFT_MOTOR) 
                self.isRunning = False

            derivate_counter = derivate_counter + 1
            if (derivate_counter % derivative_length == 0):
                error_last_r = error_r
                error_last_l = error_l
            time.sleep(dt)

        logger.info("counts %d", derivate_counter)
        logger.info("error l:%f r:%f", error_pl, error_pr)
        self.tBMotors.stopMotors()
        return(True)

    def turn_to_angle(self, target_angle=90):

        if(self.isRunning == True):
            return(False)
            time.sleep(0.01)
        self.isRunning = True
        dt = 0.009

        integral_l = integral_r= 0
        error_r = error_l = error_last_l = error_last_r = 0
        derivative_length=32
        derivate_counter=0
        MAX_PWM= 100# Percent duty cycle
        current_odo = [0, 0]
        circumference = 188.495592
        logger.info("Kp: %0.3f\tKi: %0.3f\t Kd: %0.3f", self.Kp, self.Ki, self.Kd)
        odo = 0
        duty = 255
        if target_angle > 0 :
            self.set_direction("LEFT")
            direction = -1 
        if target_angle < 0 :
            self.set_direction("RIGHT")
            direction = 1
        to_turn = target_angle*8.05 
         
        while self.isRunning == True and not self.stopper.is_set():
            pwm_r = (self.Kp*error_r) + (self.Ki * integral_r) + (self.Kd * integral_r)
            pwm_l = (self.Kp*error_l) + (self.Ki * integral_l) + (self.Kd * integral_l)
            if (pwm_l > MAX_PWM):
                pwm_l = MAX_PWM
            elif (pwm_l < -MAX_PWM):
                pwm_l = -MAX_PWM
            else:
                integral_l = integral_l + (error_l*dt)
            derivative_l = (error_l - error_last_l) / dt;

            if (pwm_r > MAX_PWM):
                pwm_r =  MAX_PWM
            elif (pwm_r < -MAX_PWM):
                pwm_r = -MAX_PWM
            else:
                integral_r = integral_r + (error_r*dt)
            derivative_r = (error_r - error_last_r) / dt;
 
            if(pwm_r > 0 ):
                self.tBMotors.setDirection(self.RIGHT_MOTOR, "FORWARD")
            if pwm_r < 0 :
                self.tBMotors.setDirection(self.RIGHT_MOTOR, "REVERSE")
            pwm_r = self.scalepwm(abs(pwm_r), 0, MAX_PWM, duty)
            
            if(pwm_l > 0 ):
                self.tBMotors.setDirection(self.LEFT_MOTOR, "FORWARD")
            if pwm_l < 0 :
                self.tBMotors.setDirection(self.LEFT_MOTOR, "REVERSE")
            pwm_l = self.scalepwm(abs(pwm_l), 0, MAX_PWM, duty)

            if derivate_counter == 0:
                pwm_l=pwm_r=255 
            self.tBMotors.runMotor(self.RIGHT_MOTOR, int(pwm_r))
            self.tBMotors.runMotor(self.LEFT_MOTOR, int(pwm_l))
            current_odo = self.serial.readserial()
            #First pass 
            if derivate_counter == 0:
                logger.info("in derivate_counter")
                setpoint_r = to_turn + current_odo[1]
                setpoint_l = to_turn + current_odo[0]
            error_r = setpoint_r - current_odo[1]
            error_l = setpoint_l - current_odo[0]
            error_pr = 100*abs(error_r/to_turn)
            error_pl = 100*abs(error_l/to_turn)
            if (error_pr < 0.5 or error_pl < 0.5):
                self.tBMotors.stopMotor(self.RIGHT_MOTOR)
                self.tBMotors.stopMotor(self.LEFT_MOTOR) 
                self.isRunning = False

            derivate_counter = derivate_counter + 1
            if (derivate_counter % derivative_length == 0):
                error_last_r = error_r
                error_last_l = error_l
            time.sleep(dt)

        logger.info("counts %d", derivate_counter)
        logger.info("error l:%f r:%f", error_pl, error_pr)
        self.tBMotors.stopMotors()
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

    def scalepwm(self, value, p_min, p_max, pwm_max ):
        return ((pwm_max-0)*(value-(p_min))/(p_max-p_min) + 0) 
    
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
