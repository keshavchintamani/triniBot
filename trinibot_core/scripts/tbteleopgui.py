#!/usr/bin/env python

import rospy
import roslib
import random
import time as Time
import Tkinter
import atexit
import tf.transformations as tftransforms


roslib.load_manifest('trinibot_core')

from geometry_msgs.msg import Twist, Pose

PI = 3.1428571429

class robotGUI_tk(Tkinter.Tk):

    def __init__(self, parent):
        global client
        Tkinter.Tk.__init__(self, parent)
        self.parent = parent
        self.initialize()
        self.pose_pub = rospy.Publisher('cmd_pos', Pose, queue_size=1)
        self.twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    def initialize(self):

        self.grid()
        self.speedmodeVar = 0
        labelStatus = Tkinter.Label(self,  text = u"Status", anchor="w", fg="white", bg ="blue")
        labelStatus.grid(column=0, row=0, sticky='EW')

        self.angleEntryVar = Tkinter.StringVar(value="30")

        labelAngle = Tkinter.Label(self, text = u"Angular", anchor="w", fg="white", bg ="blue")
        labelAngle.grid(column=0, row=1, sticky='EW')

        self.distanceEntryVar = Tkinter.StringVar(value="100")

        labelSpeed = Tkinter.Label(self, text = u"Linear", anchor="w", fg="white", bg ="blue")
        labelSpeed.grid(column=0, row=2, sticky='EW')

        distanceentry = Tkinter.Entry(self, textvariable = self.distanceEntryVar )
        distanceentry.grid(column=1, row = 2, sticky='EW')
        distanceentry.bind("<Return>", self.OnDistancePressEnter)

        angleentry = Tkinter.Entry(self, textvariable = self.angleEntryVar )
        angleentry.grid(column=1, row = 1, sticky='EW')
        angleentry.bind("<Return>", self.OnAnglePressEnter)

        self.speedmodeVar = Tkinter.IntVar()

        checkButton = Tkinter.Checkbutton(self, text=u"Rate control", variable=self.speedmodeVar, command = self.modecallback)
        checkButton.grid(column=1, row=3)

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

    def modecallback(self):
        self.speedmodeVar = not self.speedmodeVar

    def OnDistancePressEnter(self,event):
        print "You pressed  %s" % self.distanceEntryVar.get()

    def OnAnglePressEnter(self,event):
        print "You pressed  %s" % self.angleEntryVar.get()

    def OnForwardButtonClick(self):
        self.linearPlatform(1)

    def OnBackButtonClick(self):
        self.linearPlatform(-1)

    def OnLeftButtonClick(self):
        self.angularPlatform(1)

    def OnRightButtonClick(self):
        self.angularPlatform(-1)

    def OnStopButtonClick(self):
        self.linearPlatform(0)

    def linearPlatform(self, direction):
        if self.speedmodeVar == 1:
            val = Twist()
            val.linear.x = direction * int(self.distanceEntryVar.get())
            self.publishTwist(val)
        else:
            val = Pose()
            val.position.x =  direction * int(self.distanceEntryVar.get())
            self.publishPose(val)

    def angularPlatform(self, direction):
        if self.speedmodeVar == 1:
            val = Twist()
            val.angular.z = direction*int(self.angleEntryVar.get())*PI/180
            self.publishTwist(val)
        else:
            val = Pose()
            val.orientation=tftransforms.quaternion_from_euler(0,0,direction*int(self.angleEntryVar.get())*PI/180)
            self.publishPose(val)

    def publishPose(self, pose):
        self.pose_pub.publish(pose)
        rospy.loginfo(pose)

    def publishTwist(self, twist):
        self.twist_pub.publish(twist)
        rospy.loginfo(twist)

if __name__ == '__main__':
    try:
        rospy.init_node('teleop_gui', anonymous=True)
        app = robotGUI_tk(None)
        app.title('triniBot Teleoperation Controls')
        app.mainloop()

    except rospy.ROSInterruptException:
        pass
