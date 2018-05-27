#!/usr/bin/env python

import rospy
import roslib
import rosbag
import time as Time
import argparse

import sys
import csv

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

import tf.transformations as transforms
import numpy as np


Odo = Odometry()
twist_cmd = Twist()
writer = None

q_old = transforms.quaternion_from_euler(0, 0, 0)
q_current = transforms.quaternion_from_euler(0,0,0)

velocity_matrix = []
final_pose = []
final_acc = []

def handle_trinibot_pose(odom):

    global q_old
    global final_pose, final_acc
    q_old[3] = -q_old[3]
    q_current[0] = odom.pose.pose.orientation.x
    q_current[1] = odom.pose.pose.orientation.y
    q_current[2] = odom.pose.pose.orientation.z
    q_current[3] = odom.pose.pose.orientation.w

    velocity_matrix.append([odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z, \
                            odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z])
    q_rel = transforms.quaternion_multiply(q_current, q_old)
    euler = transforms.euler_from_quaternion(q_rel)

    print "%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f" % (odom.pose.pose.position.x, \
                                                      odom.pose.pose.position.y,\
                                                      odom.pose.pose.position.z, \
                                                      euler[0], euler[1], euler[2])
    final_pose = [odom.pose.pose.position.x, \
                    odom.pose.pose.position.y,\
                    odom.pose.pose.position.z, \
                    euler[0], euler[1], euler[2]]

    q_old = q_current


if __name__ == '__main__':
    #
    # pseudo unicode
    #
    # set linear and angular acceleration (x, y, th)
    # set duration
    # set output file name as comma seperated values
    #
    # send commands for the duration specified and record the odometry message
    #
    # time Vx_expected Vy_expected Wth_expected X_actual Y_actual TH_actual Vx_actual Vy_actual Wth_actual

    #Inputs x, y, th, time, fname
    rospy.init_node("covariance_logger", anonymous=False)
    if len(sys.argv) > 1:

        twist_cmd.linear.x = float(sys.argv[1])
        twist_cmd.linear.y = float(sys.argv[2])
        twist_cmd.linear.z = 0
        twist_cmd.angular.x = 0
        twist_cmd.angular.y = 0
        twist_cmd.angular.z = float(sys.argv[3])
        ttime = int(sys.argv[4])
        print "Target Twist"
        print "%f %f %f %d" %  (twist_cmd.linear.x,  twist_cmd.linear.y,  twist_cmd.angular.z, ttime)

    else:
        rospy.logerr("Arguments are %d and should be non zero", len(sys.argv))
        exit()
    vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=100)
    string_pub = rospy.Publisher("trinibot_gui/string_cmd", String, queue_size=100)

    try:
        sub=rospy.Subscriber('/trinibot/odometry', Odometry, handle_trinibot_pose)
        rospy.sleep(0.1)
        vel_pub.publish(twist_cmd)

        sleep_time = rospy.Duration(ttime)
        rospy.sleep(sleep_time)
        stop_twist = Twist()
        vel_pub.publish(stop_twist)

    finally:
        sub.unregister()
        rospy.sleep(1)
        velocity_matrix = np.array(velocity_matrix)
        #print velocities
        np.savetxt("velocities.out", velocity_matrix, fmt='%0.6f')
        #print final_pose
        #print velocity_matrix.mean(0)
        print "%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f" %(final_pose[0],final_pose[1],final_pose[2],\
                                                           final_pose[3],final_pose[4],final_pose[5], \
                                                           velocity_matrix.mean(0)[0], velocity_matrix.mean(0)[1], velocity_matrix.mean(0)[2], \
                                                           velocity_matrix.mean(0)[3], velocity_matrix.mean(0)[4], velocity_matrix.mean(0)[5])






