#!/usr/bin/env python

import rospy
import roslib
import random
import time as Time
import Tkinter
import atexit

roslib.load_manifest('trinibot_core')

from geometry_msgs.msg import TwistStamped

class robotGUI_tk(Tkinter.Tk):

    def __init__(self,parent):
        Tkinter.Tk.__init__(self, parent)
        self.parent = parent
        self.initialize()
        self.pub = rospy.Publisher('/tbteleopgui/TwistStamped', TwistStamped, queue_size=10)

    def initialize(self):
        self.grid()

        labelStatus = Tkinter.Label(self,  text = u"Status", anchor="w", fg="white", bg ="blue")
        labelStatus.grid(column=0, row=0, sticky='EW')

        self.angleEntryVar = Tkinter.StringVar(value="30")

        labelAngle = Tkinter.Label(self, text = u"Angle", anchor="w", fg="white", bg ="blue")
        labelAngle.grid(column=0, row=1, sticky='EW')

        self.distanceEntryVar = Tkinter.StringVar(value="100")

        labelSpeed = Tkinter.Label(self, text = u"Speed", anchor="w", fg="white", bg ="blue")
        labelSpeed.grid(column=0, row=2, sticky='EW')

        distanceentry = Tkinter.Entry(self, textvariable = self.distanceEntryVar )
        distanceentry.grid(column=1, row = 2, sticky='EW')
        distanceentry.bind("<Return>", self.OnDistancePressEnter)

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

    def OnDistancePressEnter(self,event):
        print "You pressed  %s" % self.distanceEntryVar.get()

    def OnAnglePressEnter(self,event):
        print "You pressed  %s" % self.angleEntryVar.get()

    def OnForwardButtonClick(self):
        self.publish_command("LINEAR",float(self.distanceEntryVar.get()))

    def OnLeftButtonClick(self):
        self.publish_command("ANGULAR",float(self.angleEntryVar.get()))

    def OnRightButtonClick(self):
        self.publish_command("ANGULAR", float(self.angleEntryVar.get())*(-1))

    def OnBackButtonClick(self):
        self.publish_command("LINEAR", float(self.distanceEntryVar.get())*(-1))

    def OnStopButtonClick(self):
        self.publish_command("STOP", 0)

    def gracefullyExit(self):
        #Stop the motors
        self.publish_command("STOP", 0)

    def publish_command(self,direction, value):

        msg = TwistStamped()
        msg.twist.linear.x = msg.twist.linear.y = msg.twist.linear.z = 0.0
        msg.twist.angular.x = msg.twist.angular.y = msg.twist.angular.z = 0.0
        if (direction == "LINEAR"):
            msg.header.frame_id= direction
            msg.twist.linear.x = value
        elif (direction == "ANGULAR"):
            msg.header.frame_id = direction
            msg.twist.angular.z = value
        elif (direction == "STOP"):
            msg.header.frame_id = direction
            msg.twist.angular.z = value

        self.pub.publish(msg)

if __name__ == '__main__':
    try:
        rospy.init_node('teleop_gui', anonymous=True)
        app = robotGUI_tk(None)
        app.title('triniBot Teleoperation Controls')
        app.mainloop()
        atexit.register(app.gracefullyExit)

    except rospy.ROSInterruptException:
	pass
