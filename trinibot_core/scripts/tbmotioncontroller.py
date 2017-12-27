#!/usr/bin/env python

# Ros node for tbmotioncontroller for tracker trinibot with 12T tracks - assume motors have a gear ratio of 150:1

import rospy
import roslib
roslib.load_manifest('trinibot_core')
import sys
import tf.transformations as transforms
import time

from tbmotorcontroller import TrackedTrinibot
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import String

wheel_diameter = 39
circumference =	122.5221135
pulses_rotation = 1800
node_name = 'trinibot_motioncontroller'

start_reading = False

def twist_callback(vel):
    #rospy.loginfo(vel)
    robot.twist(vel.linear.x, vel.linear.y, vel.angular.z)

def stop_callback(cb):
    vals = cb.data.split()
    if vals[0] == "STOP":
        robot.stop()
        start_reading = False
    elif vals[0] =="ODORESET":
        robot.reset_odo()
    elif vals[0]=="GAINS":
        vals = cb.data.split()
        try:
            kp = int(vals[1])
            ki = int(vals[3])
            kd = int(vals[2])
            rospy.loginfo("Gains set to %d %d %d",kp,kd,ki)
            robot.setgains(kp,ki,kd)
        except ValueError:
            rospy.logerror("Invalid gain value")
    else:
        rospy.logwarn("%s: Invalid string", cb)

def listener():

    global robot, start_reading
    rospy.init_node(node_name, anonymous=True)
    rospy.Subscriber('/velocity_cmd', Twist , twist_callback)
    rospy.Subscriber('/string_cmd', String, stop_callback)
    pub = rospy.Publisher('/trinibot/odometry', Odometry, queue_size= 10)
    xy = 0

    if len(sys.argv) > 1:
        robot = TrackedTrinibot(xy, sys.argv[1])
        robot.setgains(int(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4]))
    else:
        robot = TrackedTrinibot(xy, '/dev/ttyACM0')
        robot.setgains(100, 1, 1)
        print "using default serial device and gains"

    r = rospy.Rate(100)

    odom = Odometry()

    while not rospy.is_shutdown():
        #if(robot.is_running()):
        robot.get_feedback()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.child_frame_id ="base_link"
        odom.pose.pose.position.x = float(robot.serial.variables.x['value'])
        odom.pose.pose.position.y = float(robot.serial.variables.y['value'])

        q = transforms.quaternion_from_euler(0, 0, float(robot.serial.variables.theta['value']))

        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        #  axis = x,y,z, r, p, y
        odom.pose.covariance = [0.01, 0.01, 0.01, \
                                0.01, 0.01, 0.01, \
                                0.01, 0.01, 0.01, \
                                0.01, 0.01, 0.01, \
                                0.01, 0.01, 0.01, \
                                0.01, 0.01, 0.01, \
                                0.01, 0.01, 0.01, \
                                0.01, 0.01, 0.01, \
                                0.01, 0.01, 0.01, \
                                0.01, 0.01, 0.01, \
                                0.01, 0.01, 0.01, \
                                0.01, 0.01, 0.01]
        odom.twist.twist.linear.x = float(robot.serial.variables.vx['value'])
        odom.twist.twist.linear.y = float(robot.serial.variables.vy['value'])
        odom.twist.twist.angular.z = float(robot.serial.variables.omega['value'])

        odom.twist.covariance = [0.01, 0.01, 0.01, \
                                0.01, 0.01, 0.01, \
                                0.01, 0.01, 0.01, \
                                0.01, 0.01, 0.01, \
                                0.01, 0.01, 0.01, \
                                0.01, 0.01, 0.01, \
                                0.01, 0.01, 0.01, \
                                0.01, 0.01, 0.01, \
                                0.01, 0.01, 0.01, \
                                0.01, 0.01, 0.01, \
                                0.01, 0.01, 0.01, \
                                0.01, 0.01, 0.01]
        pub.publish(odom)

        r.sleep()

if __name__ == '__main__':

    listener()
    robot.exit()

