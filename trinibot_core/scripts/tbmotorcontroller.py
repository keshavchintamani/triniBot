#!/usr/bin/env python

import atexit
import threading
import sys
import time
import logging
import re
import signal
import serial
import struct

class messageLib:
    x = dict(mid=73, value=0)
    y = dict(mid=74, value=0)
    theta = dict(mid=75, value=0)
    vx = dict(mid=76, value=0)
    vy = dict(mid=77, value=0)
    omega = dict(mid=78, value=0)

class tBSerialReader():

    def __init__(self, devid):

        self.devid = devid
        self.wspeed_left = 0
        self.wspeed_right=0
        self.error_left = 0
        self.error_right = 0
        self.bytesIn = []
        self.cleanBytes = []
        self.mainSwitch = 0
        self.byteIn = 0
        self.ST = 0xaa;
        self.ET = 0xbb;
        self.STx = '\xaa'
        self.ETx = '\xbb'
        self.previousByte = 0
        self.errors = 0
        self.variables = messageLib()
        self.isUpdated = False
        self.threadstop = False
        self._startserial()

    def _startserial(self):

        print("Trying to open Serial arduino: %s ", self.devid)
        try:
            self.Serial = serial.Serial(self.devid, 115200, timeout=1)
            print("Success: %s ", self.Serial)
            time.sleep(1)
            pass
        except (IOError, ValueError):
            print("Cannot open serial device: %s", self.devid)
            raise IOError

    def ProcessMsg(self):
        b = ''.join(map(chr, self.cleanBytes[0:2]))
        msgId = struct.unpack('H', b)[0]

        if msgId == 140 or msgId == 141:
            b = ''.join(map(chr, self.cleanBytes[2:6]))
            msgValue = float(struct.unpack('L', b)[0]) / 100
        else:
            if msgId == 71 or msgId == 72:
                b = ''.join(map(chr, self.cleanBytes[2:6]))
                msgValue = float(struct.unpack('l', b)[0]) / 1000000
                print msgValue

            else:
                b = ''.join(map(chr, self.cleanBytes[2:6]))
                msgValue = struct.unpack('<f', b)[0]

        for a in dir(self.variables):
            if not a.startswith('__'):
                var = getattr(self.variables, a)
                if var['mid'] == msgId:
                    var['value'] = msgValue

        self.isUpdated = True

    def ProcessRawBytes(self):
        # clean markers
        self.cleanBytes = []
        skip = False
        checkS = 0
        for i in range(0, len(self.bytesIn)):
            if skip:
                skip = False
                continue
            if self.bytesIn[i] == self.ST or self.bytesIn[i] == self.ET:
                self.cleanBytes.append(self.bytesIn[i])
                self.skip = True
            else:
                self.cleanBytes.append(self.bytesIn[i])
            if len(self.cleanBytes) < 7:
                checkS ^= self.bytesIn[i]
        # checkSum
        checkSint = self.cleanBytes[6] | self.cleanBytes[7] << 8
        if checkS == checkSint:
            self.ProcessMsg()

    def ProcessByteIn(self):
        if self.byteIn == self.STx and self.mainSwitch == 0:
            self.mainSwitch = 1
            self.bytesIn = []
            return
        if self.byteIn == self.ETx and self.mainSwitch == 1 and self.previousByte == self.STx:
            self.mainSwitch = 0
            self.bytesIn = bytearray(self.bytesIn)
            try:
                self.ProcessRawBytes()
            except:
                print 'error'
                self.errors += 1

        if len(self.bytesIn) > 20:
            self.errors += 1
            print 'error'
            self.mainSwitch = 0
            self.bytesIn = []
            return
        else:
            self.bytesIn.append(self.byteIn)

    def readSerial(self):

        while self.Serial.inWaiting() > 0:
            self.byteIn = self.Serial.read(1)
            self.ProcessByteIn()
            self.previousByte = self.byteIn
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
        self.stopped = False

    def addnewline(self, str):
        return (str+"\n")

    def set_running(self, val):
        self.stopped = val

    def twist(self, vx, vy, w):
        command= "twist_"+str(vx)+" " + str(vy) + " " + self.addnewline(str(w))
        self.logger.info(command)
        self.serial.writeserial(command)

    def drive_at_speed(self, target_speed=100.0):
       self.set_running(True)
       command = "speed_"+ self.addnewline(str(target_speed))
       self.logger.info(command);
       self.serial.writeserial(command)
       return(True) 

    def turn_at_rate(self, target_rate=50.0):
       self.set_running(True)
       command = "spin_" + self.addnewline(str(target_rate))
       self.logger.info(command);
       self.serial.writeserial(command)
       return(True) 

    def drive_to_distance(self, target_distance=188.49):
       self.set_running(True)
       command = "goto_" + self.addnewline(str(target_distance))
       self.logger.info(command);
       self.serial.writeserial(command)
       return(True)

    def turn_to_angle(self, target_angle=45):
       self.set_running(True)
       command = "turn_" + self.addnewline(str(target_angle))
       self.logger.info(command);
       self.serial.writeserial(command)
       return(True)

    def stop(self):
       command = self.addnewline("stop_")
       self.logger.info("stop called");
       self.serial.writeserial(command)
       self.set_running(False)
       return(True)

    def setgains(self, kp, ki, kd):
       command = "gains_" + self.addnewline(str(kp)+" "+str(ki)+" "+str(kd))
       self.serial.writeserial(command)

    def get_feedback(self):
        return(self.serial.readSerial())

    def is_running(self):
        return(self.stopped)

    def reset_odo(self):
        command = self.addnewline("odoreset_")
        self.serial.writeserial(command)
        self.set_running(False)
        self.logger.info("odo reset called");

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
