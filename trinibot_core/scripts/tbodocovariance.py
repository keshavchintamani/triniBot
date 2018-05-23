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

Odo = Odometry()
twist_cmd = Twist()
writer = None

def handle_trinibot_pose(odom):
    #Do something with the odometry received
    print odom.pose.pose.orientation
    euler = transforms.euler_from_quaternion((odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,odom.pose.pose.orientation.z, odom.pose.pose.orientation.w),\
                                             axes='sxyz')
    #writer.writerow((odom.header.stamp, odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z, euler[0], euler[1], euler[2], \
     #               odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z, odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z))
    print "%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f" % (odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z, \
                                                                                                  euler[0], euler[1], euler[2], \
                    odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z, odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z)

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
        fname = str(sys.argv[5])
        print "%f %f %f %d %s" %  (twist_cmd.linear.x,  twist_cmd.linear.y,  twist_cmd.angular.z, ttime, fname )

    else:
        rospy.logerr("Arguments are %d and should be non zero", len(sys.argv))
        exit()
    vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=100)
    string_pub = rospy.Publisher("trinibot_gui/string_cmd", String, queue_size=100)

    try:
        csvfile = open(fname, 'wb')
        writer = csv.writer(csvfile)
        writer.writerow((twist_cmd.linear.x, twist_cmd.linear.y, twist_cmd.angular.z))

        rospy.Subscriber('/trinibot/odometry', Odometry, handle_trinibot_pose)

        #string_pub.publish("ODORESET")
        rospy.sleep(0.1)
        vel_pub.publish(twist_cmd)

        sleep_time = rospy.Duration(ttime)
        rospy.sleep(sleep_time)
        stop_twist = Twist()
        vel_pub.publish(stop_twist)

    finally:
        rospy.sleep(3)
        csvfile.close()


