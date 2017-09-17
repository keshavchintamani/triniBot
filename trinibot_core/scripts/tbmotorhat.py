#!/usr/bin/env python
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor
# Ros node for tbmotioncontroller for tracker trinibot with 12T tracks - assume motors have a gear ratio of 150:1

import rospy
import roslib
roslib.load_manifest('trinibot_core')

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import String

class ():

    def __init__(self, motors, i2caddress = 0x60):

        self.mh = Adafruit_MotorHAT(addr=i2caddress)
        allmotors=[1,2,3,4]
        self.user_motors=motors
        self.motor = {}
        self.motor.fromkeys(allmotors)
        for (i,x) in enumerate(self.user_motors):
            if (x < 1 or x > 4):
                return (False)
            self.motor[x] = self.mh.getMotor(x)

    def setDirection(self, mId, direction):

        # Set the direction
        if direction == "FORWARD":
            self.motor[mId].run(Adafruit_MotorHAT.FORWARD)
        elif direction == "REVERSE":
            self.motor[mId].run(Adafruit_MotorHAT.BACKWARD)
        else:
            return (False)

    def runMotor(self, mId, dutycycle):
        self.motor[mId].setSpeed(dutycycle)

    def stopMotors(self):
        print("Attemping to stop all motors")
        for mId in self.user_motors:
            self.motor[mId].run(Adafruit_MotorHAT.RELEASE)

    # recommended for auto-disabling motors on shutdown!
    def turnOffMotor(self, mId):
        print "Releasing motor %s" % mId
        self.motor[mId].run(Adafruit_MotorHAT.RELEASE)

    def cleanClose():
        self.stopMotors()


def twist_callback(vel):

    robot.setDirection(1, "FORWARD")
    robot.setDirection(3, "FORWARD")
    robot.runMotor(1, 250)
    robot.runMotor(1, 250)

def stop_callback(cb):
    if cb.data == "STOP":
        robot.cleanClose()
        start_reading = False
    else:
        rospy.logwarn("%s: Invalid string", node_name)

def listener():
    global robot, start_reading
    rospy.init_node(node_name, anonymous=True)
    rospy.Subscriber('/velocity_cmd', Twist , twist_callback)
    rospy.Subscriber('/string_cmd', String, stop_callback)
    r = rospy.Rate(60)
    rospy.spin()


if __name__ == '__main__':

    robot = MotorHatDCMotorController([1,3], 0x60)
    listener()