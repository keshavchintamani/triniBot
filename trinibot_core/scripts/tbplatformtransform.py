#!/usr/bin/env python

import rospy
import tf
import tf.transformations as transforms

from geometry_msgs.msg import Vector3Stamped, QuaternionStamped
if __name__ == '__main__':
    rospy.init_node('tbplatformtransform', anonymous=True)
    rate = rospy.Rate(100)
    broadcaster = tf.TransformBroadcaster()

    while not rospy.is_shutdown():
        broadcaster.sendTransform((47, 0, 0), transforms.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "base_camera", "base_link")
        rate.sleep()


