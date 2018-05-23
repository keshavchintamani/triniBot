#!/usr/bin/env python

# Ros node for tbmotioncontroller for tracker trinibot with 12T tracks - assume motors have a gear ratio of 150:1

import rospy
import roslib
roslib.load_manifest('trinibot_core')
import sys
import tf.transformations as transforms
import time
import math as m
from tbmotorcontroller import TrackedTrinibot
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import String

wheel_diameter = 39
circumference =	122.5221135
pulses_rotation = 1800
node_name = 'trinibot_motioncontroller'

start_reading = False

def fetch_param(name, default):
    if rospy.has_param(name):
        return rospy.get_param(name)
    else:
        print "parameter [%s] not defined. Defaulting to %s" % (name, str(default))
        return default

def twist_callback(vel):
    #rospy.loginfo(vel)
    if not (abs(m.sqrt(m.pow(vel.linear.x, 2) + m.pow(vel.linear.y, 2))) > max_lin_vel or abs(vel.angular.z) > max_ang_vel):
        try:
            robot.twist(vel.linear.x, vel.linear.y, vel.angular.z)
        except NameError:
            rospy.logerr("Callback started without data")

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
    global max_lin_vel, max_ang_vel
    rospy.init_node(node_name, anonymous=True)

    pub = rospy.Publisher('/trinibot/odometry', Odometry, queue_size= 10)
    xy = 0

    #Get parameters from server
    rospy.loginfo("Opening robot on port %s",fetch_param('/port', '/dev/ttyACM0'))
    gains= fetch_param('/pid_gains', [10, 1, 1])
    rospy.loginfo("Setting robot gains to %d %d %d", gains[0], gains[1], gains[2] )
    robot = TrackedTrinibot(xy, fetch_param('/port', '/dev/ttyACM0'))
    robot.setgains(gains[0], gains[1], gains[2])

    r = rospy.Rate(100)

    odom = Odometry()
    odom.pose.covariance = rospy.get_param("/pose_covariance")
    odom.twist.covariance = rospy.get_param("/twist_covariance")

    rospy.Subscriber('/velocity_cmd', Twist, twist_callback)
    rospy.Subscriber('/string_cmd', String, stop_callback)

    max_lin_vel= fetch_param('/max_linear_velocity', 0.1)
    max_ang_vel= fetch_param('/max_angular_velocity', 1.57)

    while not rospy.is_shutdown():
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
        odom.twist.twist.linear.x = float(robot.serial.variables.vx['value'])
        odom.twist.twist.linear.y = float(robot.serial.variables.vy['value'])
        odom.twist.twist.angular.z = float(robot.serial.variables.omega['value'])

        pub.publish(odom)

        r.sleep()

if __name__ == '__main__':

    listener()
    robot.exit()

