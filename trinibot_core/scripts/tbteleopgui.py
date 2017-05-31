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
from std_msgs.msg import String
import tf.transformations as tf_T

class robotGUI_tk(Tkinter.Tk):

    def __init__(self, parent):
        global client
        Tkinter.Tk.__init__(self, parent)
        self.parent = parent
        self.initialize()

    def initialize(self):

        self.grid()
        self.speedmodeVar = 0
        labelStatus = Tkinter.Label(self,  text = u"Status", anchor="w", fg="white", bg ="blue")
        labelStatus.grid(column=0, row=0, sticky='EW')

        self.angleEntryVar = Tkinter.StringVar(value="30")

        labelAngle = Tkinter.Label(self, text = u"Angular", anchor="w", fg="white", bg ="blue")
        labelAngle.grid(column=0, row=1, sticky='EW')

        self.distanceEntryVar = Tkinter.StringVar(value="0.01")

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
        checkButton.select()
        self.modecallback()

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

        odobutton = Tkinter.Button(self, text=u"Odometer Reset", command=self.OdoResetButtonClick)
        odobutton.grid(column=0, row=5)


    def modecallback(self):
        self.speedmodeVar = not self.speedmodeVar

    def OnDistancePressEnter(self,event):
        print "You pressed  %s" % self.distanceEntryVar.get()

    def OnAnglePressEnter(self,event):
        print "You pressed  %s" % self.angleEntryVar.get()

    def feeback_callback(self, feedback):
        rospy.loginfo("Received result: %s", feedback)

    def done_callback(self, result1, result2):
        rospy.loginfo("Received result: %s", result1)
        rospy.loginfo("Received result: %s", result2)

    def OdoResetButtonClick(self):
        rospy.loginfo("odo reset pressed")
        reset_odo = String()
        reset_odo.data = "ODORESET"
        stop_pub.publish(reset_odo)

    def OnForwardButtonClick(self):
        if self.speedmodeVar == 1:
            t = Twist()
            t.linear.x = float(self.distanceEntryVar.get())
            vel_pub.publish(t)
        else:
            p = Pose()
            p.position.x = float(self.distanceEntryVar.get())
            pose_pub.publish(p)

    def OnLeftButtonClick(self):
        if self.speedmodeVar == 1:
            t = Twist()
            t.angular.z = float(self.angleEntryVar.get())
            vel_pub.publish(t)
        else:
            p = Pose()
            p.orientation = tf_T.quaternion_from_euler(0, 0, 1 * float(self.angleEntryVar.get()), axes='sxyz')
            pose_pub.publish(p)

    def OnRightButtonClick(self):
        if self.speedmodeVar == 1:
            t = Twist()
            t.angular.z = -1*float(self.angleEntryVar.get())
            vel_pub.publish(t)
        else:
            p = Pose()
            p.orientation = tf_T.quaternion_from_euler(0, 0, -1 * float(self.angleEntryVar.get()), axes='sxyz')
            pose_pub.publish(p)

    def OnBackButtonClick(self):
        if self.speedmodeVar == 1:
            t = Twist()
            t.linear.x = -1*float(self.distanceEntryVar.get())
            vel_pub.publish(t)
        else:
            p = Pose()
            p.position.x = -1*float(self.distanceEntryVar.get())
            pose_pub.publish(p)

    def linearPlatform(self, direction):
        if self.speedmodeVar == 1:
            t = Twist()
            t.angular.z = -1*float(self.angleEntryVar.get())
            vel_pub.publish(t)
        else:
            p = Pose()
            p.orientation = tf_T.quaternion_from_euler(0,0,-1*float(self.angleEntryVar.get()),axes='sxyz')
            pose_pub.publish(p)

    def angularPlatform(self, direction):
        if self.speedmodeVar == 1:
            t = Twist()
            t.linear.x = -1*float(self.distanceEntryVar.get())
            vel_pub.publish(t)
        else:
            p = Pose()
            p.position.x = -1*float(self.distanceEntryVar.get())
            pose_pub.publish(p)

    def OnStopButtonClick(self):
        stop = String()
        stop.data = "STOP"
        stop_pub.publish(stop)

    def publishTwist(self, twist):
        self.twist_pub.publish(twist)
        rospy.loginfo(twist)

if __name__ == '__main__':
    try:
        rospy.init_node('teleop_gui', anonymous=True)
        app = robotGUI_tk(None)
        app.title('triniBot Teleoperation Controls')
        pose_pub = rospy.Publisher("trinibot_gui/position_cmd", Pose, queue_size=1)
        vel_pub = rospy.Publisher("trinibot_gui/velocity_cmd", Twist, queue_size=1)
        stop_pub = rospy.Publisher("trinibot_gui/string_cmd", String, queue_size=1)
        app.mainloop()

    except rospy.ROSInterruptException:
        pass
