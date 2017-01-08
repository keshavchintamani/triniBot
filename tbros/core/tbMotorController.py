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
            pass 
    #    return ((self.old_encoder_left, self.old_encoder_right))



#Documentation
#TrackedTriniBot is a specialized tracked platform using the 150:1 micrometal gearmotors
#It assumes certain dimensions of the wheels for its odometry
#Changes to the wheels or transmision should be reflected in the parameters
class TrackedTrinibot():

    def __init__(self,stopper, tacho_cb, encoder_dev = '/dev/ttyUSB0', motorcontrol_dev = 0x60):

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

    def openlogger(self, path="logs/"):
    #    self.logfilepath = path + "log_" + str(self.logfileindex) + ".dat"
         return(open("logs/" + "log_" + str(self.logfileindex) + ".dat", "w")
)

    def drive_at_speed(self, direction, target_speed=300):
        self.set_direction(direction)
        if(self.isRunning == True):
            #self.isRunning = False
            return(False)
            time.sleep(0.01)
        self.isRunning = True
        dt = 0.01
        integral_l = integral_r= 0
        current_speed_l = current_speed_r = error_last_l = error_last_r = 0
        odo_l = 0
        odo_r = 0
        # track travel in centimeters/encoder pulse is 0.006806 for 150:1 and 0.0102 for 100:1
        cm_pulse= 0.010471#006806 #0.006806
        # degrees/encoder pulse is 06.2 for 150:1 and 0.3 for 100:1 - which one is it!!!???
        deg_pulse=0.2
        derivative_length=64
        derivate_counter=0
        MAX_PWM= 100 # Percent duty cycle
        log_array = np.empty((0,6), dtype=float)
        while self.isRunning == True and not self.stopper.is_set():
            error_l = target_speed - current_speed_l
            error_r = target_speed - current_speed_r
            #average the error
            error_percent_l = abs(error_l)/target_speed*100
            error_percent_r = abs(error_r)/target_speed*100
            pwm_l = (self.Kp*error_l) + (self.Ki * integral_l) + (self.Kd * integral_l)
            pwm_r = (self.Kp*error_r) + (self.Ki * integral_r) + (self.Kd * integral_r)

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

            pwm_s_l = self.scalepwm(pwm_l)
            pwm_s_r = self.scalepwm(pwm_r)

            self.tBMotors.runMotor(self.LEFT_MOTOR, int(pwm_s_l))
            self.tBMotors.runMotor(self.RIGHT_MOTOR, int(pwm_s_r))
            current_odo = self.serial.readserial()
            #Problem is here - on the first pass last_odo_l = 0 so diff_l > target_distance
            if(derivate_counter > 1):
                diff_l =  (current_odo[0] - last_odo_l)
                diff_r =  (current_odo[1] - last_odo_r)
                odo_l = odo_l + diff_l
                odo_r = odo_r + diff_r 
                try:
                        self.tach_callback((current_speed_l, current_speed_r,odo_l*cm_pulse, odo_r*cm_pulse))
                except  ValueError:
                        logger.info("did you forget the callback?")

                current_speed_l = deg_pulse * (diff_l)/dt
                current_speed_r = deg_pulse * (diff_r)/dt
            last_odo_l = current_odo[0]
            last_odo_r = current_odo[1]

            #if error_percent_l < 2 or error_percent_r < 2 :
            #    self.isRunning = False
            log_array = np.append(log_array, [self.Kp, self.Kd, target_speed, current_speed_l, current_speed_r, \
            odo_l*cm_pulse, odo_r*cm_pulse])

            #            str(target_distance) + "\t" + str(odo_l*cm_pulse) + "\t" + \
            #            str(error_l) + "\t" + str(pwm_l) +"\n"
            derivate_counter = derivate_counter + 1
            if (derivate_counter % derivative_length == 0):
                error_last_l = error_l
                error_last_r = error_r
            time.sleep(dt)

        self.Stop()
        np.save("logs/"+str(derivate_counter) + ".txt", log_array)
        logfile.close()
        return(True)


    def drive_to_distance(self, direction, target_distance=10):

        self.set_direction(direction)
        if(self.isRunning == True):
            #self.isRunning = False
            return(False)
            time.sleep(0.01)
        self.isRunning = True
        logfile = self.openlogger() #open("logs/" + "log_" + str(self.logfileindex) + ".dat", "w")
        dt = 0.01
        integral_l = integral_r= 0
        error_last_l = error_last_r = 0
        odo_l = 0
        odo_r = 0
        # track travel in centimeters/encoder pulse is 0.006806 for 150:1 and 0.0102 for 100:1
        cm_pulse= 0.010471#006806 #0.006806
        # degrees/encoder pulse is 06.2 for 150:1 and 0.3 for 100:1 - which one is it!!!???
        deg_pulse=0.2
        derivative_length=64
        derivate_counter=0
        MAX_PWM= 50 # Percent duty cycle
        log_array = np.empty((0,5), dtype=float)
        while self.isRunning == True and not self.stopper.is_set():
            error_l = target_distance - odo_l*cm_pulse
            error_r = target_distance - odo_r*cm_pulse
            #average the error
            error_percent_l = abs(error_l)/target_distance*100
            error_percent_r = abs(error_r)/target_distance*100
            pwm_l = (self.Kp*error_l) + (self.Ki * integral_l) + (self.Kd * integral_l)
            pwm_r = (self.Kp*error_r) + (self.Ki * integral_r) + (self.Kd * integral_r)

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

            pwm_s_l = self.scalepwm(pwm_l)
            pwm_s_r = self.scalepwm(pwm_r)

            self.tBMotors.runMotor(self.LEFT_MOTOR, int(pwm_s_l))
            self.tBMotors.runMotor(self.RIGHT_MOTOR, int(pwm_s_r))

            current_odo = self.serial.readserial()
            #Problem is here - on the first pass last_odo_l = 0 so diff_l > target_distance
            if(derivate_counter > 1):
                diff_l =  (current_odo[0] - last_odo_l)
                diff_r =  (current_odo[1] - last_odo_r)
            
                odo_l = odo_l + diff_l
                odo_r = odo_r + diff_r 
            
                logger.info("diff: %f %f %d", error_percent_r, error_r,  odo_r) 
                current_speed_l = deg_pulse * (current_odo[0] - last_odo_l)/dt
                current_speed_r = deg_pulse * (current_odo[1] - last_odo_r)/dt
            last_odo_l = current_odo[0]
            last_odo_r = current_odo[1]

            if error_percent_l < 2 or error_percent_r < 2 :
                self.isRunning = False
            log_array = np.append(log_array, [self.Kp, self.Kd, target_distance, odo_l*cm_pulse, error_l, pwm_l])

            #logstring = str(self.Kp)+"\t"+str(self.Kd)+"\t" + \
            #            str(target_distance) + "\t" + str(odo_l*cm_pulse) + "\t" + \
            #            str(error_l) + "\t" + str(pwm_l) +"\n"
            derivate_counter = derivate_counter + 1
            #logger.info(logstring)
            if (derivate_counter % derivative_length == 0):
                error_last_l = error_l
                error_last_r = error_r
            time.sleep(dt)

        self.Stop()
        logger.info("target:%fM - desired:%fM", float(odo_l*cm_pulse), float(target_distance))
        np.save("logs/"+str(derivate_counter) + ".txt", log_array)
        logfile.close()
        return(True)

    def turn_to_angle(self, direction, target_angle=90):

        self.set_direction(direction)
        if(self.isRunning == True):
            #self.isRunning = False
            return(False)
            time.sleep(0.01)
        self.isRunning = True
        logfile = self.openlogger() #open("logs/" + "log_" + str(self.logfileindex) + ".dat", "w")
        dt = 0.01
        integral_l = integral_r= 0
        last_odo_l = last_odo_r = error_last_l = error_last_r = 0
        odo_l = odo_r = 0
        Odo=self.serial.readserial()
        start_odo_l = int(Odo[0])
        start_odo_r = int(Odo[1])

        base_deg_pulse = 0.08074534161

        derivative_length=64
        derivate_counter=0
        while self.isRunning == True and not self.stopper.is_set():

            current_odo = self.serial.readserial()
            odo_l = abs(current_odo[0] - start_odo_l)
            odo_r = abs(current_odo[1] - start_odo_r)

            current_speed_l = base_deg_pulse * (current_odo[0] - last_odo_l)/dt
            current_speed_r = base_deg_pulse * (current_odo[1] - last_odo_r)/dt

            error_l = target_angle - odo_l*base_deg_pulse
            error_r = target_angle - odo_r*base_deg_pulse

            #average the error
            error_percent = (abs(error_l)/target_angle*100 + abs(error_r)/target_angle*100)/2

            pwm_l = (self.Kp*error_l) + (self.Ki * integral_l) + (self.Kd * integral_l)
            pwm_r = (self.Kp*error_r) + (self.Ki * integral_r) + (self.Kd * integral_r)

            if (pwm_l > 100):
                pwm_l = 100
            elif (pwm_l < -100):
                pwm_l = -100
            else:
                integral_l = integral_l + (error_l*dt)

            derivative_l = (error_l - error_last_l) / dt;

            if (pwm_r > 100):
                pwm_r = 100
            elif (pwm_r < -100):
                pwm_r = -100
            else:
                integral_r = integral_r + (error_r*dt)

            derivative_r = (error_r - error_last_r) / dt;

            pwm_l = self.scalepwm(pwm_l)
            pwm_r = self.scalepwm(pwm_r)

            self.tBMotors.runMotor(self.LEFT_MOTOR, int(pwm_l))
            self.tBMotors.runMotor(self.RIGHT_MOTOR, int(pwm_r))

            last_odo_l = current_odo[0]
            last_odo_r = current_odo[1]

            if error_percent < 2:
                self.isRunning = False
            #logstring = str(self.Kp)+"\t"+str(self.Ki)+"\t" + \
            #            str(target_angle) + "\t" + str(odo*cm_pulse) + "\t" + \
            #            str(error_percent) + "\t" + str(pwm_l) +"\t" + \
            #            str(pwm_r) +  "\n"

            derivate_counter = derivate_counter + 1

            if (derivate_counter % derivative_length == 0):
                # derivate_length has passed
                error_last_l = error_l
                error_last_r = error_r

            time.sleep(dt)

        self.Stop()
        logger.info("target:%fM - desired:%fM", float(odo*base_deg_pulse), float(target_angle))
        logfile.write(logstring)
        logfile.close()

        return(True)

    def setKp(self, kp):
            self.Kp = float(kp)

    def setKi(self, ki):
            self.Ki = float(ki)

    def setKd(self, kd):
            self.Kd = float(kd)


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

    myrobot = TrackedTrinibot(Stopper, "/dev/ttyACM0")
    threads.append(myrobot)

    handler = SignalHandler(Stopper, threads)
    handler.SetRobot(myrobot)
    signal.signal(signal.SIGINT, handler)

    KP=[0.001, 0.01, 0.1, 1, 10, 100]
    KD=[0.001, 0.01, 0.1, 1, 10, 100]# 0.5, 0.6, 0.7, 0.8, 0.9]
    for Kp in KP:
        for Kd in KD:
            logger.info("Running motor at Kp: %f and Ki: %f", Kp, Kd)  
            myrobot.setKd(Kd)
            myrobot.setKp(Kp)
            status = myrobot.drive_to_distance("FORWARD", 19)
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
    print len(sys.argv)
    if sys.argv[1] == 'd':
        myrobot.drive_to_distance("FORWARD", int(sys.argv[2])) 
    elif sys.argv[1] == 's' and len(sys.argv) == 5:
        myrobot.setKd(float(sys.argv[3]))
        myrobot.setKp(float(sys.argv[2]))
        myrobot.drive_at_speed("FORWARD", int(sys.argv[2]))
    elif sys.argv[1] == 'a':
            myrobot.turn_to_angle("LEFT", int(sys.argv[2]))
    else:
        logger.info("Invalid number of arguments")
   
    myrobot.Cleanup()

def robot_data(val):
        print val
def exithandler():

    logger.info("Goodbye!")

if __name__ == '__main__':
    atexit.register(exithandler)
    if len(sys.argv) == 1: 
       main_automation()
    else:
       main_cli_args()
