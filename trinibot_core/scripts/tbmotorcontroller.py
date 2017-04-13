#!/usr/bin/env python

import atexit
import threading
from threading import Timer
import sys
import time
import serial
import logging
import re
import signal

class tBSerialReader():

    def __init__(self, devid):

        self.devid = devid
        self.wspeed_left = 0
        self.wspeed_right=0
        self.error_left = 0
        self.error_right = 0
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
    
    def readserial(self):

        line = self.Serial.readline()
        res = re.findall("[-+]?\d+[\.]?\d*", line)
        try:
            if len(res) > 1:
                return (res)

        except ValueError:
            logger.error("Value error exception parsing serial data")
            pass

    def writeserial(self, message):
        try:
            self.Serial.write(message);
            return(True);
        except (IOError, ValueError):
            print("Cannot open serial device: %s", self.devid)
            raise IOError
        return(False);

#Documentation
#Serial wrapper for the Teensy motor controller for Trinibot
class TrackedTrinibot():


    def __init__(self, feedback, serial = "/dev/ttyUSB0"):
        global logger
        self.Ki = 1
        self.Kp = 10
        self.Kd = 1
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
        self.callback = feedback
        self.timerFlag = True

    def addnewline(self, str):
        return (str+"\n")

    def drive_at_speed(self, target_speed=100.0):
       command = "speed_"+ self.addnewline(str(target_speed))
       self.logger.info(command);
       self.serial.writeserial(command)
       return(True) 

    def turn_at_rate(self, target_rate=50.0):
       command = "spin_" + self.addnewline(str(target_rate))
       self.logger.info(command);
       self.serial.writeserial(command)
       return(True) 

    def drive_to_distance(self, target_distance=188.49):
       command = "goto_" + self.addnewline(str(target_distance))
       self.logger.info(command);
       self.serial.writeserial(command)
       return(True)

    def turn_to_angle(self, target_angle=45):
       command = "turn_" + self.addnewline(str(target_angle))
       self.logger.info(command);
       self.serial.writeserial(command)
       return(True)

    def stop(self):
       command = self.addnewline("stop_")
       self.serial.writeserial(command)
       return(True)

    def setgains(self, kp, ki, kd):
       command = "gains_" + self.addnewline(str(kp)+" "+str(ki)+" "+str(kd))
       self.serial.writeserial(command)

    def get_feedback(self):
        return(self.serial.readserial())

    def exit(self):
        self.stop()


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
