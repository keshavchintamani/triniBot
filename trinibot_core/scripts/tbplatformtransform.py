#!/usr/bin/env python

import rospy
import tf
import tf2_ros
import geometry_msgs.msg
import nav_msgs.msg

#Republishes trinibot odometry in the trinibot odom frame

def handle_trinibot_pose(odo):
    
    br = tf2_ros.TransformBroadcaster()
    po = geometry_msgs.msg.TransformStamped()
    po.header.stamp = rospy.Time.now()
    po.header.frame_id = "world"
    po.child_frame_id = "odom"

    po.transform.translation = odo.pose.pose.position
    po.transform.rotation = odo.pose.pose.orientation
    rospy.loginfo("broadcasting tranform %s", po)
    br.sendTransform(po)

if __name__ == '__main__':
    rospy.init_node('trinibot_tf_publisher', anonymous=True)
    rate = rospy.Rate(100)
    rospy.Subscriber('/trinibot/odometry',
                     nav_msgs.msg.Odometry,
                     handle_trinibot_pose)
    rospy.spin()

   