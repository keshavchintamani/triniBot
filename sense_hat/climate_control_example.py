from sense_hat import SenseHat
import time
from datetime import datetime
from evdev import InputDevice, categorize, ecodes,list_devices
from select import select
import threading
from SenseLogger import SenseLogger
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor


MOTOR = 2
s=SenseHat()
logger = SenseLogger(s)
mh = Adafruit_MotorHAT(addr=0x61)

atexit.register(stopMotor)

def stopMotor():
    print("Wow! thats much better!")
    mh.getMotor(2).run(Adafruit.MotorHAT.RELEASE)

def runMotor():
    print("Too hot! Cooling myself down!")
    myMotor=mh.getMotor(MOTOR)
    myMotor.run(Adafruit.MotorHAT.FORWARD)
    myMotor.setSpeed(150)

while(True):    
    v=s.get_humidity()
    print("%f" % s.get_humidity())
    if(v>40 and v<50):
        s.load_image("/home/pi/triniBot/assets/8x8_red_smiley.png",True)
    elif(v>=50 ):
        s.load_image("/home/pi/triniBot/assets/8x8_red_happy.png",True)
        stopMotor()
    elif(v>=60 and v<100):
        s.load_image("/home/pi/triniBot/assets/8x8_red_angry.png",True)
        runMotor()

    key_press, run = logger.check_input()  

    if key_press:
        logger.on_key_press()
        print("Writing data to log")
