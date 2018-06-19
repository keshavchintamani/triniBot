#!/usr/bin/env python

import rospy
import tf
import tf2_ros
import geometry_msgs.msg
import nav_msgs.msg

#Republishes trinibot odometry in the trinibot odom frame

def handle_trinibot_pose(odo):
    
    br = tf2_ros.TransformBroadcaster()
    #Transform of baselink in a fixed odometry frame. note that the odometry frame will move if reset
    time = rospy.Time.now()

    map_tr = geometry_msgs.msg.TransformStamped()
    map_tr.header.stamp = time
    map_tr.header.frame_id = "map"
    map_tr.child_frame_id = "odom"

    map_tr.transform.translation.x = 0.0
    map_tr.transform.translation.y = 0.0
    map_tr.transform.rotation.w = 1

    br.sendTransform(map_tr)


    baselink_tr = geometry_msgs.msg.TransformStamped()
    baselink_tr.header.stamp = time
    baselink_tr.header.frame_id = "odom"
    baselink_tr.child_frame_id = "base_link"

    baselink_tr.transform.translation = odo.pose.pose.position
    baselink_tr.transform.translation.z = 0.0195
    baselink_tr.transform.rotation = odo.pose.pose.orientation

    br.sendTransform(baselink_tr)

    #Transform from base_link to depth camera
    depth_camera_tr = geometry_msgs.msg.TransformStamped()
    depth_camera_tr.header.stamp = time
    depth_camera_tr.header.frame_id = "base_link"
    depth_camera_tr.child_frame_id = "depth_frame"

    depth_camera_tr.transform.translation.x = 0.073024
    depth_camera_tr.transform.translation.z = 0.057846 + 0.023
    depth_camera_tr.transform.translation.y = -0.013
    depth_camera_tr.transform.rotation.x = 0.707
    depth_camera_tr.transform.rotation.z = 0
    depth_camera_tr.transform.rotation.y = 0
    depth_camera_tr.transform.rotation.w = 0.707

    br.sendTransform(depth_camera_tr)

    # Transform from base_link to depth camera
    rgb_camera_tr = geometry_msgs.msg.TransformStamped()
    rgb_camera_tr.header.stamp = time
    rgb_camera_tr.header.frame_id = "depth_frame"
    rgb_camera_tr.child_frame_id = "rgb_frame"

    rgb_camera_tr.transform.translation.x = 0#0.073024
    rgb_camera_tr.transform.translation.z = -0.026#.0574 + 0.023
    rgb_camera_tr.transform.translation.y = 0#.026
    rgb_camera_tr.transform.rotation.x = 0
    rgb_camera_tr.transform.rotation.z = 0
    rgb_camera_tr.transform.rotation.y = 0
    rgb_camera_tr.transform.rotation.w = 1

    br.sendTransform(rgb_camera_tr)

    # Transform from base_link to imu
    imu_tr = geometry_msgs.msg.TransformStamped()
    imu_tr.header.stamp = time
    imu_tr.header.frame_id = "base_link"
    imu_tr.child_frame_id = "trinibot_imu"

    #TODO Check
    imu_tr.transform.translation.x = 0.008
    imu_tr.transform.translation.z = 0.084788
    imu_tr.transform.translation.y = 0.0
    imu_tr.transform.rotation.x = 0
    imu_tr.transform.rotation.y = 0
    imu_tr.transform.rotation.z = 0
    imu_tr.transform.rotation.w = 1

    br.sendTransform(imu_tr)

    #Transform from base_link to lazer_scan
    lazer_tr = geometry_msgs.msg.TransformStamped()
    lazer_tr.header.stamp = time
    lazer_tr.header.frame_id = "depth_frame"
    lazer_tr.child_frame_id = "lazer_frame"

    lazer_tr.transform.translation.x = 0#0.073024
    lazer_tr.transform.translation.z = 0#0.057846 + 0.023
    lazer_tr.transform.translation.y = 0#-0.013
    lazer_tr.transform.rotation.x = -0.707
    lazer_tr.transform.rotation.z = 0
    lazer_tr.transform.rotation.y = 0
    lazer_tr.transform.rotation.w = 0.707

    br.sendTransform(lazer_tr)


if __name__ == '__main__':
    rospy.init_node('trinibot_tf_publisher', anonymous=True)
    rate = rospy.Rate(100)
    rospy.Subscriber('/trinibot/odometry',
                     nav_msgs.msg.Odometry,
                     handle_trinibot_pose)
    rospy.spin()

   
