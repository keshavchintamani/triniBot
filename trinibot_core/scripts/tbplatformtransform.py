#!/usr/bin/env python

import rospy
import tf
import tf2_ros
import geometry_msgs.msg
import nav_msgs.msg

#Republishes trinibot odometry in the trinibot odom frame

def handle_trinibot_pose(odo):
    
    br = tf2_ros.TransformBroadcaster()
    odom_tr = geometry_msgs.msg.TransformStamped()
    odom_tr.header.stamp = rospy.Time.now()
    odom_tr.header.frame_id = "map"
    odom_tr.child_frame_id = "odom"

    #Assumes the odometry is measured from mid point between the two tracks
    odom_tr.transform.translation = odo.pose.pose.position
    odom_tr.transform.translation.z = 0.0195
    odom_tr.transform.rotation = odo.pose.pose.orientation

    br.sendTransform(odom_tr)

    baselink_tr = geometry_msgs.msg.TransformStamped()
    baselink_tr.header.stamp = rospy.Time.now()
    baselink_tr.header.frame_id = "odom"
    baselink_tr.child_frame_id = "base_link"

    baselink_tr.transform.translation.x = baselink_tr.transform.translation.z = \
    baselink_tr.transform.translation.y = 0
    baselink_tr.transform.rotation.x = baselink_tr.transform.rotation.z = \
    baselink_tr.transform.rotation.y = 0
    baselink_tr.transform.rotation.w = 1

    br.sendTransform(baselink_tr)

    camera_tr = geometry_msgs.msg.TransformStamped()
    camera_tr.header.stamp = rospy.Time.now()
    camera_tr.header.frame_id = "base_link"
    camera_tr.child_frame_id = "camera"

    camera_tr.transform.translation.x = 0.043
    camera_tr.transform.translation.z = 0.0855
    camera_tr.transform.translation.y = 0.0245
    camera_tr.transform.rotation.x = camera_tr.transform.rotation.z = \
    camera_tr.transform.rotation.y = 0
    camera_tr.transform.rotation.w = 1

    br.sendTransform(camera_tr)

if __name__ == '__main__':
    rospy.init_node('trinibot_tf_publisher', anonymous=True)
    rate = rospy.Rate(100)
    rospy.Subscriber('/trinibot/odometry',
                     nav_msgs.msg.Odometry,
                     handle_trinibot_pose)

    rospy.spin()

   