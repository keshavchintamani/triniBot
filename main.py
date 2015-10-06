__author__ = 'kc'

import threading
import time as Time
import random as Random
import Queue
import serial
from triniBotMotorController import triniBotMotorController
import atexit

ticker = 0
encoderQueue = Queue.Queue()


def cleanClose():
    print('Attempting to close motors');
    m1.turnOffMotors();
    m2.turnOffMotors();

class EncoderThread(threading.Thread):

    def __init__(self, threadID, name, q):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.q = q
        #self.Serial = serial.Serial('/dev/ttyUSB0', 9600);
        Random.random()
        Random.seed()

    def run(self):
        #Get
        while (True):
            #line = self.Serial.readline()
            line = Random.randint(1,10)
            self.q.put(line)
            Time.sleep(0.01)

threadLock = threading.Lock()

motors = [1 , 3]
#Start a worker thread polling on serial interface for encoders from arduino
eThread = EncoderThread('1', 'encoder-getter', encoderQueue)
eThread.start()

#start a worker thread driving the motor
#m1 = triniBotMotorController(1, encoderQueue)
#m1.setDirection("FORWARD")
#m1.start()
#m1.runMotor(100)

#start a worker thread driving the motor
m2 = triniBotMotorController(motors, encoderQueue)
m2.start()

m2.setDirection(1, "FORWARD")
m2.setDirection(3, "FORWARD")

while(1):
    m2.runMotor(1, 100)
    Time.sleep(0.2)
    m2.runMotor(2, 100)


#m2.start()
#m2.runMotor(100)

atexit.register(cleanClose)
