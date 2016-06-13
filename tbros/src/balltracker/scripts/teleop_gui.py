#!/usr/bin/env python

import rospy
import roslib
import random
import time as Time
import Tkinter

roslib.load_manifest('balltracker')

from balltracker.msg import command

class robotGUI_tk(Tkinter.Tk):

    def __init__(self,parent):
        Tkinter.Tk.__init__(self, parent)
        self.parent = parent
        self.initialize()
        self.pub = rospy.Publisher('/tbmotionplanner/command', command, queue_size=10)


    def initialize(self):
        self.grid()

        labelStatus = Tkinter.Label(self,  text = u"Status", anchor="w", fg="white", bg ="blue")
        labelStatus.grid(column=0, row=0, sticky='EW')

        self.angleEntryVar = Tkinter.StringVar(value="30")

        labelAngle = Tkinter.Label(self, text = u"Angle", anchor="w", fg="white", bg ="blue")
        labelAngle.grid(column=0, row=1, sticky='EW')


        self.speedEntryVar = Tkinter.StringVar(value="100")

        labelSpeed = Tkinter.Label(self, text = u"Speed", anchor="w", fg="white", bg ="blue")
        labelSpeed.grid(column=0, row=2, sticky='EW')

        speedentry = Tkinter.Entry(self, textvariable = self.speedEntryVar )
        speedentry.grid(column=1, row = 2, sticky='EW')
        speedentry.bind("<Return>", self.OnSpeedPressEnter)

        angleentry = Tkinter.Entry(self, textvariable = self.angleEntryVar )
        angleentry.grid(column=1, row = 1, sticky='EW')
        angleentry.bind("<Return>", self.OnAnglePressEnter)

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

    def OnSpeedPressEnter(self,event):
        print "You pressed  %s" % self.speedEntryVar.get()

    def OnAnglePressEnter(self,event):
        print "You pressed  %s" % self.angleEntryVar.get()

    def OnForwardButtonClick(self):
        self.publish_command("LOW","FORWARD",self.speedEntryVar.get() , 0)

    def OnLeftButtonClick(self):
        self.publish_command("LOW","LEFT",self.speedEntryVar.get() , self.angleEntryVar.get())

    def OnRightButtonClick(self):
        self.publish_command("LOW","RIGHT",self.speedEntryVar.get() , self.angleEntryVar.get())

    def OnBackButtonClick(self):
        self.publish_command("LOW","REVERSE",self.speedEntryVar.get(), 0)

    def OnStopButtonClick(self):
        self.publish_command("LOW","STOP",self.speedEntryVar.get(), 0)

    def gracefullyExit(self):
        #Stop the motors
        self.publish_command("LOW","STOP",self.speedEntryVar.get(), 0)

    def publish_command(self,priority, direction, speed, angle):

        msg = command()
        msg.direction = str(direction)
        msg.speed = int(speed)
        msg.angle = float(angle)
        msg.priority = str(priority)
        rospy.loginfo("Test: Publishing: %s", msg)
        self.pub.publish(msg)

if __name__ == '__main__':
    try:
        rospy.init_node('teleop_gui', anonymous=True)

        app = robotGUI_tk(None)
        app.title('triniBot UI')
        app.mainloop()
        #atexit.register(app.gracefullyExit)

    except rospy.ROSInterruptException:
	pass
