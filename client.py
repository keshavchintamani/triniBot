import socket
import sys
import atexit
import Tkinter
HOST, PORT = "192.168.1.9", 9999
#data = " ".join(sys.argv[1:])


class robotGUI_tk(Tkinter.Tk):
    def __init__(self,parent):
        Tkinter.Tk.__init__(self, parent)
        self.parent = parent
        self.initialize()
        # Create a socket (SOCK_STREAM means a TCP socket)
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((HOST, PORT))
        finally:
            return None

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
        self.sock.send("TB_DRIVE_FORWARD" + "\n")
        received = self.sock.recv(1024)
        print "You clicked the forward button!"
        print "Received: {}".format(received)
       

    def OnLeftButtonClick(self):
        self.sock.send("TB_TURN_LEFT"+ "\n")
        received = self.sock.recv(1024)
        print "You clicked the left button!"
        print "Received: {}".format(received)

    def OnRightButtonClick(self):
        self.sock.send("TB_TURN_RIGHT"+ "\n")
        received = self.sock.recv(1024)
        print "You clicked the right button!"
        print "Received: {}".format(received)

    def OnBackButtonClick(self):
        self.sock.send("TB_DRIVE_BACK"+ "\n")
        received = self.sock.recv(1024)
        print "You clicked the back button!"
        print "Received: {}".format(received)
        
    def OnStopButtonClick(self):
        self.sock.send("TB_DRIVE_STOP"+ "\n")
        received = self.sock.recv(1024)
        print "You clicked the stop button!"
        print "Received: {}".format(received)

    def gracefullyExit():
        self.sock.close()
     


# init the GUI
if __name__ == "__main__":
    app = robotGUI_tk(None)
    app.title('triniBot UI')
    app.mainloop()
    atexit.register(app.gracefullyExit)
    


