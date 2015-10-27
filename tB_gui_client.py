import socket
import sys
import atexit
import Tkinter
from zmq_publisher import zmqPub

ip="localhost"
port="5556"
if(len(sys.argv)>2):
    ip=argv[1]
    port=argv[2]

#Define the main topics to be published on 0mq
topic = "tB_TOPIC_COMMAND"

class robotGUI_tk(Tkinter.Tk):
    def __init__(self,parent):
        Tkinter.Tk.__init__(self, parent)
        self.parent = parent
        self.initialize()
        self.pub = zmqPub("GUI_pub", ip, port)
        
        
    def initialize(self):
        self.grid()


        self.videoEntryVar = Tkinter.StringVar(value="rtsp://192.168.1.9:8554/unicast")
        
       
        labelVid = Tkinter.Label(self,  text = u"Video", anchor="w", fg="white", bg ="blue")
        labelVid.grid(column=0, row=0, sticky='EW')

        self.speedEntryVar = Tkinter.StringVar(value="100")

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
        self.pub.publish(topic,"TB_DRIVE_FORWARD "+self.speedEntryVar.get())

    def OnLeftButtonClick(self):
        self.pub.publish(topic,"TB_TURN_LEFT "+self.speedEntryVar.get())
      
    def OnRightButtonClick(self):
        self.pub.publish(topic,"TB_TURN_RIGHT "+self.speedEntryVar.get())
        
    def OnBackButtonClick(self):
        self.pub.publish(topic,"TB_DRIVE_BACK "+self.speedEntryVar.get())

        
    def OnStopButtonClick(self):
        self.pub.publish(topic,"TB_DRIVE_STOP")

    def gracefullyExit():
        self.pub.teardown()
     


# init the GUI
if __name__ == "__main__":
    app = robotGUI_tk(None)
    app.title('triniBot UI')
    app.mainloop()
    atexit.register(app.gracefullyExit)
    


