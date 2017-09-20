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

    start_reading = False
    if (vel.linear.x < 0 or vel.linear.x > 0) and (vel.angular.z < 0 or vel.angular.z > 0):
        rospy.logerr("%s cannot execute linear and angular velocities simultaneously")
        return

    if vel.linear.x < 0 or vel.linear.x > 0:
        robot.drive_at_speed(vel.linear.x)
        start_reading = True
    elif vel.angular.z < 0 or vel.angular.z > 0:
        robot.turn_at_rate(vel.angular.z)
        start_reading = True

def stop_callback(cb):
    if cb.data == "STOP":
        robot.stop()
        start_reading = False
    elif cb.data =="ODORESET":
        robot.reset_odo()
    else:
        rospy.logwarn("%s: Invalid string", node_name)

def listener():
    global robot, start_reading
    rospy.init_node(node_name, anonymous=True)
    rospy.Subscriber('/velocity_cmd', Twist , twist_callback)
    rospy.Subscriber('/string_cmd', String, stop_callback)
    pub = rospy.Publisher('/trinibot/odometry', Odometry, queue_size= 10)
    xy = 0
    robot = TrackedTrinibot(xy, sys.argv[1])
    #Set gains to 10, 1, 1
    robot.setgains(int(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4]))
    r = rospy.Rate(60)
    odom_old = Odometry()
    odom = Odometry()

    while not rospy.is_shutdown():
        if(robot.is_running()):
            res = robot.get_feedback()
            try:
                arr = [float(s) for s in res.split()]
                if len(arr) < 6:
                    continue

                odom.pose.pose.position.x = arr[0]
                odom.pose.pose.position.y = arr[1]
                q = transforms.quaternion_from_euler(0, 0, arr[2])
                odom.pose.pose.orientation.x = q[0]
                odom.pose.pose.orientation.y = q[1]
                odom.pose.pose.orientation.z = q[2]
                odom.pose.pose.orientation.w = q[3]
                odom.twist.twist.linear.x = arr[3]
                odom.twist.twist.linear.y = arr[4]
                odom.twist.twist.angular.z = arr[5]
                pub.publish(odom)
                odom_old = odom
            except ValueError, e:
                print "error", e, "on value", s
                continue
        else:
            pub.publish(odom_old)

        r.sleep()
        # get the gyro values

if __name__ == '__main__':

    listener()
    robot.exit()

