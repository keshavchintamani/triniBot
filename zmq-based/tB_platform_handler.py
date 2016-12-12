#!/usr/bin/python

from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor

import time as Time
import atexit
import threading
import random as Random
import Queue
import serial
from sense_hat import SenseHat
from zmq_subscriber import zmqSub
import sys
import picamera
import time
from subprocess import call
import subprocess


if(len(sys.argv)<2):
    ip="localhost"
    port=5556
    print "Insufficient args so defaulting to {}:{}".format(ip,port)
else:
    ip=sys.argv[1]
    port=sys.argv[2]
    print "Subscribing to {}:{}".format(ip,port)


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

#A thread that reads encoder data from Arduino over Serial
class tBSensorCapture(threading.Thread):

    def __init__(self, threadID, q, senseHat):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.q = q
        self.senseHat=senseHat

    def run(self):
        #Get
        while (True):
            print("Getting sensors")

#Implementation of Two wheeled robot assuming LEFT motor is 1 and RIGHT Motor is 3
# now includes steering control

# refactor class name to 2WheelRobot
class tBController():
    def __init__(self, leftMotorId, rightMotorId, steering):
        self.motors=[leftMotorId, rightMotorId, steering]
        self.tBMotors = tBMotorController(self.motors)
        self.isRunning=False
        self.lock = threading.Lock()
        
    def DriveForward(self,speed):

        self.tBMotors.setDirection(LEFT_MOTOR, "FORWARD")
        self.tBMotors.setDirection(RIGHT_MOTOR, "FORWARD")
        while(self.isRunning ==True):
            self.tBMotors.runMotor(LEFT_MOTOR, int(speed[0]))
            self.tBMotors.runMotor(RIGHT_MOTOR, int(speed[0]))
            time.sleep(0.01)
        print("Exiting drive Forward")
                                                
    def DriveBackward(self,speed):
        self.tBMotors.setDirection(LEFT_MOTOR, "REVERSE")
        self.tBMotors.setDirection(RIGHT_MOTOR, "REVERSE")
        while(self.isRunning ==True):
            self.tBMotors.runMotor(LEFT_MOTOR, int(speed[0]))
            self.tBMotors.runMotor(RIGHT_MOTOR, int(speed[0]))
            time.sleep(0.01)
        print("Exiting drive Backward")

    def steerLeft(self,speed):

        self.tBMotors.setDirection(STEERING, "FORWARD")
        while(self.isRunning ==True):
            self.tBMotors.runMotor(STEERING, int(speed[0]))
            #self.tBMotors.runMotor(RIGHT_MOTOR, int(speed[0]))
            time.sleep(0.01)
        print("Exiting Turn Left")

    def steerRight(self,speed):

        self.tBMotors.setDirection(STEERING, "REVERSE")
        while(self.isRunning ==True):
            self.tBMotors.runMotor(STEERING, int(speed[0]))
            #self.tBMotors.runMotor(RIGHT_MOTOR, int(speed[0]))
            time.sleep(0.01)
        print("Exiting Turn Left")

        
    def TurnLeft(self,speed):
        
        self.tBMotors.setDirection(LEFT_MOTOR, "REVERSE")
        self.tBMotors.setDirection(RIGHT_MOTOR, "FORWARD")
        while(self.isRunning ==True):
            self.tBMotors.runMotor(LEFT_MOTOR, int(speed[0]))
            self.tBMotors.runMotor(RIGHT_MOTOR, int(speed[0]))
            time.sleep(0.01)
        print("Exiting Turn Left")

    def TurnRight(self,speed):
        self.tBMotors.setDirection(LEFT_MOTOR, "FORWARD")
        self.tBMotors.setDirection(RIGHT_MOTOR, "REVERSE")
        while(self.isRunning ==True):
            self.tBMotors.runMotor(LEFT_MOTOR, int(speed[0]))
            self.tBMotors.runMotor(RIGHT_MOTOR, int(speed[0]))
            time.sleep(0.01)
        print("Exiting Turn Right")

    def StopCurrentLoop(self):
        #Makes an existing while loop in thread exit
        self.isRunning=False
        time.sleep(0.01)
        #Set to true for next command
        self.isRunning=True

    def Stop(self):
        self.tBMotors.stopMotors()
    
    def GetIfRunning(self):
        return(self.isRunning)
    
                                 
#Wrapper and helper functions for Adafruit motorHat
class tBMotorController():

    def __init__(self, motors):

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
            
        #self.sensorq = sensorq
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
STEERING = 4
ticker = 0

mainloop=True
topicfilter = "tB_TOPIC_COMMAND"
#Create a subscriber
print "Collecting command updates from any publisher..."
sub = zmqSub("command_subscriber", ip, port, topicfilter)

def CleanUpandExit():
    print("Cleaning up and closing... Goodbye!")
    sub.teardown()
    mainloop=False
    

def initTriniBot():

    global sHat, tBMotion 
    #Start the video server
    subprocess.Popen(["h264_v4l2_rtspserver"])
    #Get a reference to sensehat
    #sHat = SenseHat()
    #Create a platform controller for the motors
    tBMotion = tBController(LEFT_MOTOR, RIGHT_MOTOR, STEERING)

    atexit.register(CleanUpandExit)
    
    #sHat.show_message("Start...", 0.1, [255,0,0], [0,0,0])
    #time.sleep(0.5)
    #i=3
    #while(i>0):
    #    sHat.show_letter(str(i), [Random.randint(0,255),Random.randint(0,255),Random.randint(0,255)],[0,0,0])
    #    time.sleep(1)
    #    i=i-1
    #sHat.clear()
    return(True)


if __name__ == "__main__":

    global sHat, tBMotion 

    if(initTriniBot()):
        print "Successfully initialized TriniBot"
    else:
        print "Something went wrong :-(..."
        sys.exit()
    while(mainloop==True): 
        topic, message = sub.subscribe()
        print "Received %s %s" % (topic, message)
        command=message[0]
        params=message[1:len(message)]
      
        if(command == "TB_DRIVE_FORWARD"):

            tBMotion.StopCurrentLoop()
            t = threading.Thread(target=tBMotion.DriveForward, args=[params])
            t.start()
            
        elif(command == "TB_TURN_LEFT"):

            tBMotion.StopCurrentLoop()
            t = threading.Thread(target=tBMotion.TurnLeft, args=[params])
            t.start()
        
        elif(command == "TB_TURN_RIGHT"):

            tBMotion.StopCurrentLoop()
            t = threading.Thread(target=tBMotion.TurnRight, args=[params])
            t.start()
            
        elif(command == "TB_DRIVE_BACK"):

            tBMotion.StopCurrentLoop()
            t = threading.Thread(target=tBMotion.DriveBackward, args=[params])
            t.start()
            
        elif(command == "TB_DRIVE_STOP"):

            tBMotion.Stop()
        elif(command == "TB_DRIVE_HUBLEFT"):
            tBMotion.StopCurrentLoop()
            t = threading.Thread(target=tBMotion.steerLeft, args=[params])
            t.start()
        elif(command == "TB_DRIVE_HUBRIGHT"):
            tBMotion.StopCurrentLoop()
            t = threading.Thread(target=tBMotion.steerRight, args=[params])
            t.start()
            
        #Read at 1KHz
        time.sleep(0.001)
        
