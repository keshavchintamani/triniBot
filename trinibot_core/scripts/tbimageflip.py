#!/usr/bin/env python

import message_filters
from sensor_msgs.msg import Image, CameraInfo
import rospy
import numpy as np
import cv2
import cv_bridge

camerarepub = rospy.Publisher('/trinibot_camera/image_flipped/image_raw', Image, queue_size=10)
camerainfopub = rospy.Publisher('/trinibot_camera/image_flipped/camera_info', CameraInfo, queue_size=10)


def Callback(image, caminfo):
    image = bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
    image = cv2.flip(image, -1)
    image = bridge.cv2_to_imgmsg(image, encoding = "bgr8")
    camerarepub.publish(image)
    camerainfopub.publish(caminfo)


if __name__ == '__main__':

    bridge = cv_bridge.CvBridge()
    rospy.init_node('tbmotioncontroller', anonymous=True)
    info_sub  = message_filters.Subscriber('/camera_info', CameraInfo)
    image_sub = message_filters.Subscriber('/image_raw', Image)

    ts = message_filters.TimeSynchronizer([image_sub, info_sub], 10)
    ts.registerCallback(Callback)
    rospy.spin()


