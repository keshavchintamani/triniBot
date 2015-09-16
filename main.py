__author__ = 'kc'

import threading
import time as Time
import random as Random
import Queue
import serial
#import triniBotMotorController

ticker = 0
encoderQueue = Queue.Queue()

class FakeController(threading.Thread):

    def __init__(self, name, q):
        threading.Thread.__init__(self)
        self.name = name
        self.q = q
    def Initialize(self, init):
        self.init = init
    def turnOffMotor(self):
        self.turnoff = True

    def run(self):
         while(True):
             self.val = self.q.get()
             print 'Got %s' % self.val
             Time.sleep(0.01)


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

    def __init__(self, threadID, name, q):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.q = q
        #self.Serial = serial.Serial('/dev/ttyUSB0', 9600);
        Random.random()

    def run(self):
        #Get
        while (True):
            #line = self.Serial.readline()
            line = 10
            self.q.put(line)
            print 'Set %s' % line
            Time.sleep(0.01)

threadLock = threading.Lock()

#Start a worker thread polling on serial interface for encoders from arduino
eThread = EncoderThread('1', 'encoder-getter', encoderQueue)
eThread.start()

#start a worker thread driving the motor
motor1 = triniBotMotorController(1)
m1Thread = MotorThread('2', motor1, encoderQueue)
#m1T= FakeController("hello", encoderQueue)
m1Thread.start()



#while (True):
#   if not encoderQueue.empty():
#        val = encoderQueue.get()
#       print "Outputting: %s " % val



atexit.register(triniBot.turnOffMotor)

