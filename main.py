__author__ = 'kc'

import threading
import time as Time
import random as Random
import Queue
import serial
import triniBotMotorController

ticker = 0
encoderQueue = Queue.Queue()

class FakeController:

    def __init__(self, name):
        self.name = name
    def Initialize(self, init):
        self.init = init
    def turnOffMotor(self):
        self.turnoff = True

class MotorThread(threading.Thread):

    def __init__(self, tId,controller, q) :
        threading.Thread.__init__(self)
        self.threadID = tId
        self.motorControl = controller
        self.motorControl
        self.q = q
    def run(self):
        while(True):
            self.motorControl
        #Do something here with the motor

class EncoderThread(threading.Thread):

    def __init__(self, threadID, name, counter, q):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.counter = counter
        self.name = name
        self.q = q
        self.Serial = serial.Serial('/dev/ttyUSB0', 9600);
        Random.random()

    def run(self):
        #Get
        while (True):
            line = self.Serial.readline()
            self.q.put(line)

threadLock = threading.Lock()

#Start a worker thread polling on serial interface for encoders from arduino
eThread = EncoderThread('1', 'encoder-getter', encoderQueue)
eThread.run()

#start a worker thread driving the motor
motor1 = triniBotMotorController(1)
m1Thread = MotorThread('2', motor1, encoderQueue)
m1Thread.run()



#while (True):
#   if not encoderQueue.empty():
#        val = encoderQueue.get()
#       print "Outputting: %s " % val



atexit.register(triniBot.turnOffMotor)

