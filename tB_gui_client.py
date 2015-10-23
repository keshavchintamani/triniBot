import Tkinter

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
            m2.runMotor(3, i)
        while(self.notinterrupted == True):
            m2.runMotor(1, int(self.speedEntryVar.get()))
            #Time.sleep(0.2)
            m2.runMotor(3, int(self.speedEntryVar.get()))
            

    def OnLeftButtonClick(self):
        print "You clicked the button!"

    def OnRightButtonClick(self):
        print "You clicked the button!"

    def OnBackButtonClick(self):
        print "You clicked the button!"
        m2.setDirection(1, "REVERSE")
        m2.setDirection(3, "REVERSE")

        for i in range(30, int(self.speedEntryVar.get())):
            m2.runMotor(1, i)
            Time.sleep(0.2)
            m2.runMotor(3, i)

    def OnStopButtonClick(self):
        print "You clicked the button!"
        m2.stopMotors()
# init the GUI
if __name__ == "__main__":
    app = robotGUI_tk(None)
    app.title('triniBot UI')
    app.mainloop()
