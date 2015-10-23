#!/usr/bin/python

# Simple DC motor controller script for Adafruit motor hat
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor

import time as Time
import atexit
import threading
import random as Random
import Queue
import serial
from sense_hat import SenseHat

import SocketServer
#import picamera
import time


#TODO
#Ugly ugly code
# single server instance and threaded for both sensors and motors
#Server handler for sensor data requests
class tBTCPMotorHandler(SocketServer.BaseRequestHandler):
    """
    The RequestHandler class for our server.

    It is instantiated once per connection to the server, and must
    override the handle() method to implement communication to the
    client.
    """
 
    def handle(self):
        # self.request is the TCP socket connected to the client
        self.data = self.request.recv(1024).strip()
        print "{} wrote:".format(self.data)
        #print self.data
        req = self.data.split()
        if(len(req) < 1):
             self.request.sendall("Sorry cant process empty commands!")
        #Handles incoming requests
        self.processRequest(req)
        

    def processRequest(self, request):

        command = request[0]
        print "{} is requested".format(command)
        if(command == "TB_INIT"):
            t = threading.Thread(target=initTriniBot)
            t.start()
            self.request.sendall("TB_STATUS OK")
        elif(command == "TB_DRIVE_FORWARD"):
            tBMotion.DriveForward(100)
            self.request.sendall("TB_STATUS OK")
        elif(command == "TB_TURN_LEFT"):
            tBMotion.TurnLeft(80)
            self.request.sendall("TB_STATUS OK")
        elif(command == "TB_TURN_RIGHT"):
            tBMotion.TurnRight(80)
            self.request.sendall("TB_STATUS OK")
        elif(command == "TB_DRIVE_BACK"):
            tBMotion.DriveBackward(100)
            self.request.sendall("TB_STATUS OK")
        elif(command == "TB_DRIVE_STOP"):
            self.request.sendall("TB_STATUS OK")
        else:
            self.request.sendall("TB_ERROR 0")

#TODO
#Ugly ugly code
# single server instance and threaded for both sensors and motors
#Server handler for sensor data requests
class tBTCPSensorHandler(SocketServer.BaseRequestHandler):
    """
    The RequestHandler class for our server.

    It is instantiated once per connection to the server, and must
    override the handle() method to implement communication to the
    client.
    """

    def handle(self):
        # self.request is the TCP socket connected to the client
        self.data = self.request.recv(1024).strip()
        print "{} wrote:".format(self.client_address[0])
        print self.data

        req = self.data.split()
        if(len(req) < 1):
             self.request.sendall("Sorry cant process empty commands!")
        #Handles incoming requests
        self.processRequest(req)

    def processRequest(self, request):

        command = request[0]
        if(command == "TB_GET_ENVIRONMENTAL"):
            self.request.sendall("TB_STATUS OK")
        elif(command == "TB_GET_INERTIAL"):
             self.request.sendall("TB_STATUS OK")
        else:
             self.request.sendall("TB_ERROR 0")

#A thread that reads encoder data from Arduino over Serial
class tBEncoderCapture(threading.Thread):

    def __init__(self, threadID, q):
        threading.Thread.__init__(self)
        self.threadID = threadID
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

#Wrapper and helper functions for Adafruit motorHat
class tBController():

    def __init__(self):
        self.motors=[LEFT_MOTOR, RIGHT_MOTOR]
        self.tBMotors = tBMotorController(self.motors, sensorQueue)
        self.isRunning=False

    def DriveForward(self,speed):

        self.isRunning=True;
        self.tBMotors.setDirection(LEFT_MOTOR, "FORWARD")
        self.tBMotors.setDirection(RIGHT_MOTOR, "FORWARD")
        #Might need to run these in threads
        while(self.isRunning ==True):
            self.tBMotors.runMotor(LEFT_MOTOR, speed)
            self.tBMotors.runMotor(RIGHT_MOTOR, speed)
                                                
    def DriveBackward(self,speed):
        self.tBMotors.setDirection(LEFT_MOTOR, "REVERSE")
        self.tBMotors.setDirection(RIGHT_MOTOR, "REVERSE")
        #Might need to run these in threads
        self.tBMotors.runMotor(LEFT_MOTOR, speed)
        self.tBMotors.runMotor(RIGHT_MOTOR, speed)

    def TurnLeft(self,speed):
        self.tBMotors.setDirection(LEFT_MOTOR, "FORWARD")
        self.tBMotors.setDirection(RIGHT_MOTOR, "REVERSE")
        #Might need to run these in threads
        self.tBMotors.runMotor(LEFT_MOTOR, speed)
        self.tBMotors.runMotor(RIGHT_MOTOR, speed)

    def TurnRight(self,speed):
        self.tBMotors.setDirection(LEFT_MOTOR, "REVERSE")
        self.tBMotors.setDirection(RIGHT_MOTOR, "FORWARD")
        #Might need to run these in threads
        self.tBMotors.runMotor(LEFT_MOTOR, speed)
        self.tBMotors.runMotor(RIGHT_MOTOR, speed)

    def Stop(self):
        self.isRunning=False;
        stopMotors()
                                 
    #Wrapper and helper functions for Adafruit motorHat
class tBMotorController():

    def __init__(self, motors, sensorq):

        self.mh = Adafruit_MotorHAT(addr=0x60)
        allmotors=[1,2,3,4]
        self.user_motors=motors;
        self.motor = {}
        self.motor.fromkeys(allmotors);
        for (i,x) in enumerate(self.user_motors):
            print"Initializing Motor %s" % x
            if (x < 1 or x > 4):
                print "ID out of range"
                return (False)
            self.motor[x] = self.mh.getMotor(x)
            
        self.sensorq = sensorq
        self.haltRequest = False
        self.speed = 0;
        #TODO
        #Run thread to get encoders from Arduino
        #self.encoderq = Queue.Queue()
        #tBEncoders = tBEncoderCapture("encoder-capture-thread", encoderq)

    def setDirection(self, mId, direction):

        # Set the direction
        if direction == "FORWARD":
            print "Driving Forward Motor %s " % mId
            self.motor[mId].run(Adafruit_MotorHAT.FORWARD)
        elif direction == "REVERSE":
            print "Driving Backward Motor %s " % mId
            self.motor[mId].run(Adafruit_MotorHAT.BACKWARD)
        else:
            print "Invalid direction"
            return (False)

    def runMotor(self, mId, speed):

        print "In run motor"
        self.motor[mId].setSpeed(speed)
        #print "In run motor 2"
        #wheel_speed = self.encoderq.get()
        #print "In run motor 3"
        #print "Speed = %s" % wheel_speed
        #previous_error = 0
        #integral = 0
        #PID controoler when everything is ready
        #while(1)
            #get time 
            #error = setpoint - wheel_speed
            #integral = integral + error*dt
            #derivative = (error - previous_error)/dt
            #output = Kp*error + Ki*integral + Kd*derivative
            #previous_error = error
                
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


LEFT_MOTOR = 1
RIGHT_MOTOR = 3

threadLock = threading.Lock()
sensorQueue = Queue.Queue()
s = SenseHat()
ticker = 0

isInitialized=False;

tBMotion = tBController();

def initTriniBot():
    #Initial a motor controller
    
    s.show_message("Initializing...", 0.1, [255,0,0], [0,0,0])
    time.sleep(0.5)
    i=3
    while(i>0):
        s.show_letter(str(i), [Random.randint(0,255),Random.randint(0,255),Random.randint(0,255)],[0,0,0])
        time.sleep(1)
        i=i-1
    s.clear()


