__author__ = 'kc'

import threading
import time as Time
import random as Random
import Queue
import serial
from triniBotMotorController import triniBotMotorController
import atexit
import Tkinter


ticker = 0
encoderQueue = Queue.Queue()

class robotGUI_tk(Tkinter.Tk):
    def __init__(self,parent):
        Tkinter.Tk.__init__(self, parent)
        self.parent = parent
        self.initialize()
            

    def initialize(self):
        self.grid()


        self.videoEntryVar = Tkinter.StringVar()
        
       
        labelVid = Tkinter.Label(self,  text = u"Video", anchor="w", fg="white", bg ="blue")
        labelVid.grid(column=0, row=0, sticky='EW')

        self.speedEntryVar = Tkinter.StringVar()

        labelSpeed = Tkinter.Label(self, text = u"Speed", anchor="w", fg="white", bg ="blue")
        labelSpeed.grid(column=0, row=1, sticky='EW')

        
        videoentry = Tkinter.Entry(self, textvariable = self.videoEntryVar )
        videoentry.grid(column=1, row = 0, sticky='EW')
        videoentry.bind("<Return>", self.OnVideoPressEnter)
        speedentry = Tkinter.Entry(self, textvariable = self.speedEntryVar )
        speedentry.grid(column=1, row = 1, sticky='EW')
        speedentry.bind("<Return>", self.OnSpeedPressEnter)
        
               
        self.grid()
        
        fbutton = Tkinter.Button(self, text=u"forward", command = self.OnForwardButtonClick)
        fbutton.grid(column=3, row = 1)

        lbutton = Tkinter.Button(self, text=u"left" , command = self.OnLeftButtonClick)
        lbutton.grid(column=2, row = 2)

        sbutton = Tkinter.Button(self, text=u"Stop!", command = self.OnStopButtonClick)
        sbutton.grid(column=3, row = 2)

        rbutton = Tkinter.Button(self, text=u"right", command = self.OnRightButtonClick)
        rbutton.grid(column=4, row = 2)

        bbutton = Tkinter.Button(self, text=u"back", command = self.OnBackButtonClick)
        bbutton.grid(column=3, row = 3)

    def OnVideoPressEnter(self,event):
        print "You pressed  %s" % self.videoEntryVar.get()
            
    def OnSpeedPressEnter(self,event):
        print "You pressed  %s" % self.speedEntryVar.get()
            
    def OnForwardButtonClick(self):
        m2.setDirection(1, "FORWARD")
        m2.setDirection(3, "FORWARD")

        for i in range(30, int(self.speedEntryVar.get())):
            m2.runMotor(1, i)
            #Time.sleep(0.2)
            m2.runMotor(3, i)print "You clicked the button!"

    def OnLeftButtonClick(self):
        print "You clicked the button!"

    def OnRightButtonClick(self):
        print "You clicked the button!"

    def OnBackButtonClick(self):
        print "You clicked the button!"

    def OnStopButtonClick(self):
        print "You clicked the button!"    
        

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
    
# init the GUI
if __name__ == "__main__":
    app = robotGUI_tk(None)
    app.title('triniBot UI')
    app.mainloop()
    




m2.stopMotors()

m2.setDirection(1, "REVERSE")
m2.setDirection(3, "REVERSE")

for i in range(0, 100):
    m2.runMotor(1, i)
    Time.sleep(0.2)
    m2.runMotor(3, i)

m2.stopMotors()

    


#m2.start()
#m2.runMotor(100)

atexit.register(cleanClose)
