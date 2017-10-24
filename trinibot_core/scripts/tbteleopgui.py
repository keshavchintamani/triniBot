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

        self.angleEntryVar = Tkinter.StringVar(value="0.39275")

        labelAngle = Tkinter.Label(self, text = u"Angular", anchor="w", fg="white", bg ="blue")
        labelAngle.grid(column=0, row=1, sticky='EW')

        self.distanceEntryVar = Tkinter.StringVar(value="0.1")

        labelSpeed = Tkinter.Label(self, text = u"Linear", anchor="w", fg="white", bg ="blue")
        labelSpeed.grid(column=0, row=2, sticky='EW')

        label_kp = Tkinter.Label(self, text=u"Kp", anchor="w", fg="white", bg="blue")
        label_kp.grid(column=0, row=3, sticky='EW')
        label_ki= Tkinter.Label(self, text=u"Ki", anchor="w", fg="white", bg="blue")
        label_ki.grid(column=0, row=4, sticky='EW')
        label_kd = Tkinter.Label(self, text = u"Kd", anchor="w", fg="white", bg ="blue")
        label_kd.grid(column=0, row=5, sticky='EW')


        distanceentry = Tkinter.Entry(self, textvariable = self.distanceEntryVar )
        distanceentry.grid(column=1, row = 2, sticky='EW')
        #distanceentry.bind("<Return>", self.OnDistancePressEnter)

        angleentry = Tkinter.Entry(self, textvariable = self.angleEntryVar )
        angleentry.grid(column=1, row = 1, sticky='EW')
        #angleentry.bind("<Return>", self.OnAnglePressEnter)

        self.Kp = Tkinter.StringVar(value="100")
        self.Ki = Tkinter.StringVar(value="1")
        self.Kd = Tkinter.StringVar(value="1")

        kpentry = Tkinter.Entry(self, textvariable = self.Kp)
        kpentry.grid(column=1, row=3, sticky='EW')
        #kpentry.bind("<Return>", self.OnPiDEntered)

        kientry = Tkinter.Entry(self, textvariable=self.Ki)
        kientry.grid(column=1, row=4, sticky='EW')
        #kientry.bind("<Return>", self.OnPiDEntered)

        kdentry = Tkinter.Entry(self, textvariable=self.Kd)
        kdentry.grid(column=1, row=5, sticky='EW')
        #kdentry.bind("<Return>", self.OnPiDEntered)

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

        gainsbutton = Tkinter.Button(self, text=u"Update Gains", command=self.SendGains)
        gainsbutton.grid(column=0, row=6)

        odobutton = Tkinter.Button(self, text=u"Odometer Reset", command=self.OdoResetButtonClick)
        odobutton.grid(column=0, row=7)



    def modecallback(self):
        self.speedmodeVar = not self.speedmodeVar

#    def OnDistancePressEnter(self,event):
#        print "You pressed  %s" % self.distanceEntryVar.get()

#   def OnAnglePressEnter(self,event):
#       rospy.loginfo("A", result1)

#    def OnPiDEntered(self, event):

    def feeback_callback(self, feedback):
        rospy.loginfo("Received result: %s", feedback)

    def done_callback(self, result1, result2):
        rospy.loginfo("Received result: %s", result1)
        rospy.loginfo("Received result: %s", result2)

    def OdoResetButtonClick(self):
        rospy.loginfo("odo reset pressed")
        reset_odo = String()
        reset_odo.data = "ODORESET"
        string_pub.publish(reset_odo)

    def SendGains(self):

        set_gains  = String()
        set_gains.data = "GAINS " + str(self.Kp.get()) + " " + str(self.Ki.get()) + " " + str(self.Kd.get())
        rospy.loginfo("Sending gains " + set_gains.data)
        string_pub.publish(set_gains)

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
        string_pub.publish(stop)

    def publishTwist(self, twist):
        self.twist_pub.publish(twist)
        rospy.loginfo(twist)

    def exitApp(self):
        self.destroy();

def shutdownHook():
    app.exitApp()


if __name__ == '__main__':
    try:
        rospy.init_node('teleop_gui', anonymous=True)
        app = robotGUI_tk(None)
        app.title('triniBot Teleoperation Controls')
        pose_pub = rospy.Publisher("trinibot_gui/position_cmd", Pose, queue_size=1)
        vel_pub = rospy.Publisher("trinibot_gui/velocity_cmd", Twist, queue_size=1)
        string_pub = rospy.Publisher("trinibot_gui/string_cmd", String, queue_size=1)
        rospy.on_shutdown(shutdownHook)
        app.mainloop()



    except rospy.ROSInterruptException:
        pass
