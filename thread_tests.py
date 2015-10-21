

import threading
import time as Time
import random as Random
import Queue
import serial
#from triniBotMotorController import triniBotMotorController
import atexit
import Tkinter


#Class to hold sensor values
class SensorData():
    def __init__(self):
        self.Temperature = 0;
        self.Humidity = 0;
        self.Pressure = 0;
        self.Acceleration = [];
        self.Bearing = 0;
        self.Orientation_deg =[];
        

    def getOrientation(self, units):
        if(units = 'RAD'):
            return piSense.get_orientation_radians()
        elif (units = 'DEF'):
            return piSense.get_orientation_degrees()

    def getAcceleration(self, units):
        return self.Acceleration;
    
    def getBearing(self):
        return self.Bearing;

    def getTemperature(self)
        return self.Temperature;

    def getPressure(self)
        return self.Pressue;

    def getHumidity(self)
        return self.Humidity;
        
#GUI builder
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
        #Start thread to update GUI with sensor data
        self.GUIUpdateHandler = GUIUpdateThread('2', 'GUIUpdateThread-1', sensorValuesHandler)
        self.GUIUpdateHandler.start()
        

    def OnVideoPressEnter(self,event):
        print "You pressed  %s" % self.videoEntryVar.get()
            
    def OnSpeedPressEnter(self,event):
        print "You pressed  %s" % self.speedEntryVar.get()
            
    def OnForwardButtonClick(self):
        print "You pressed  %s" % self.speedEntryVar.get()

    def OnLeftButtonClick(self):
        print "You clicked the button!"

    def OnRightButtonClick(self):
        print "You clicked the button!"

    def OnBackButtonClick(self):
        print "You clicked the button!"

    def OnStopButtonClick(self):
        print "You clicked the button!"

class GUIUpdateThread(threading.Thread):

    def __init__(self, threadID, name, q):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.q = q
        Random.seed()

    def setLabels(labels):
        

    def run(self):
        while (True):
                sensorVals = self.q.get();
                print "In %s got %s" % (self.name, sensorVals.temperature)
 

class SensorThread(threading.Thread):

    def __init__(self, threadID, name, q):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.q = q
        Random.seed()

    def run(self):
        while (True):
            s = SensorData();
            #Get sensors from Global SenseHat object
            s.temperature = Random.random()
            s.Humidity = Random.random()
            s.Acceleration = Random.random()
            s.Pressure = Random.random()
            self.q.put(s)            
            Time.sleep(0.01)

#Global sensor data object which gets updated in the GUI thread
sensorData = SensorData();
#Our global reference to the PiSense
#piSense = SenseHat();
sensorValuesHandler = Queue.Queue()
St = SensorThread('1','SensorThread', sensorValuesHandler)
St.start()



# init the GUI
if __name__ == "__main__":
    app = robotGUI_tk(None)
    app.title('triniBot UI')
    app.mainloop()
    
