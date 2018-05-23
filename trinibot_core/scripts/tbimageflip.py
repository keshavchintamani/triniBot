#!/usr/bin/env python

import message_filters
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
import rospy
import numpy as np
import cv2
import cv_bridge

camera_comppub = rospy.Publisher('/trinibot_camera/image_flipped/compressed', CompressedImage, queue_size=1)
camerainfopub = rospy.Publisher('/trinibot_camera/image_flipped/camera_info', CameraInfo, queue_size=1)
camera_uncomppub = rospy.Publisher('/trinibot_camera/image_flipped/raw', Image, queue_size=1)

def Callback(image, caminfo):
    image = bridge.compressed_imgmsg_to_cv2(image)
    image = cv2.flip(image, -1)
    msg = CompressedImage()
    msg = bridge.cv2_to_compressed_imgmsg(image, dst_format= "jpeg")
    msg.header.stamp = rospy.Time.now()
    msg.format = "jpeg"
    camera_comppub.publish(msg)
    msg = Image()
    msg = bridge.cv2_to_imgmsg(image, encoding="passthrough")
    msg.header.stamp = rospy.Time.now()
    camera_uncomppub.publish(msg)

if __name__ == '__main__':

    bridge = cv_bridge.CvBridge()
    rospy.init_node('tbmotioncontroller', anonymous=True)
    info_sub  = message_filters.Subscriber('/camera_info', CameraInfo)
    image_sub = message_filters.Subscriber('/image', CompressedImage)

    ts = message_filters.TimeSynchronizer([image_sub, info_sub], 1)
    ts.registerCallback(Callback)
    rospy.spin()


