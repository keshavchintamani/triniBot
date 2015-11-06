from sense_hat import SenseHat
import time
from datetime import datetime
from evdev import InputDevice, categorize, ecodes,list_devices
from select import select
import threading
from SenseLogger import SenseLogger

s=SenseHat()
logger = SenseLogger(s)

while(True):
    
    v=s.get_humidity()
    
    #print("%f" % s.get_humidity())
    if(v>40 and v<50):
        s.load_image("/home/pi/triniBot/assets/8x8_red_happy.png",True)
    elif(v>=50 and v<60):
        s.load_image("/home/pi/triniBot/assets/8x8_red_smiley.png",True)
    elif(v>=60 and v<100):
        s.load_image("/home/pi/triniBot/assets/8x8_red_angry.png",True)

    key_press, run = logger.check_input()  

    if key_press:
        logger.on_key_press()
        print("Writing data to log")
    
        
                     
        
    
