#!/usr/bin/env python

import rospy
import roslib
import random
import time as Time
import Tkinter
import atexit
import tf_conversions as conversions
import actionlib
from trinibot_core.msg import  move_trinibotAction, move_trinibotGoal

roslib.load_manifest('trinibot_core')

from geometry_msgs.msg import TwistStamped, PoseStamped

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

    def pushGoal(self, objective, value):
        #client.cancel_goal()
        goal = move_trinibotGoal()
        goal.objective = objective
        goal.value = value
        rospy.loginfo(goal)
        client.send_goal(goal,done_cb = self.done_callback, feedback_cb=self.feeback_callback)

    def feeback_callback(self, feedback):
        rospy.loginfo("Received result: %s", feedback)

    def done_callback(self, result1, result2):
        rospy.loginfo("Received result: %s", result1)
        rospy.loginfo("Received result: %s", result2)


    def OnForwardButtonClick(self):
        if self.speedmodeVar == 1:
            objective = "speed"
        else:
            objective = "goto"
        self.pushGoal(objective, int(self.distanceEntryVar.get()))

    def OnLeftButtonClick(self):
        if self.speedmodeVar == 1:
            objective = "spin"
        else:
            objective = "turn"
        self.pushGoal(objective, int(self.angleEntryVar.get()))

    def OnRightButtonClick(self):
        if self.speedmodeVar == 1:
            objective = "spin"
        else:
            objective = "turn"
        self.pushGoal(objective, -1 * int(self.angleEntryVar.get()))

    def OnBackButtonClick(self):
        if self.speedmodeVar == 1:
            objective = "speed"
        else:
            objective = "goto"
        self.pushGoal(objective, -1*int(self.distanceEntryVar.get()))

    def OnStopButtonClick(self):
        self.pushGoal("stop", 0)


if __name__ == '__main__':
    try:
        rospy.init_node('teleop_gui', anonymous=True)
        client = actionlib.SimpleActionClient('tbmotioncontroller', move_trinibotAction)
        client.wait_for_server()
        app = robotGUI_tk(None)
        app.title('triniBot Teleoperation Controls')
        app.mainloop()

    except rospy.ROSInterruptException:
        pass
