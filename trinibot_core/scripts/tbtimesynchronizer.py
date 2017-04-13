#!/usr/bin/env python

import message_filters
from sensor_msgs.msg import Image, CameraInfo
import rospy


camerapub = rospy.Publisher('/trinibot_framesync/image_raw', Image, queue_size=10)
camerainfopub = rospy.Publisher('/trinibot_framesync/camera_info', CameraInfo, queue_size=10)

def callback(image, camera_info):
    camerapub.publish(image)
    camerainfopub.publish(camera_info)


if __name__ == '__main__':

    rospy.init_node('tbmotioncontroller', anonymous=True)
    image_sub = message_filters.Subscriber('/image', Image)
    info_sub = message_filters.Subscriber('/camera_info', CameraInfo)

    ts = message_filters.TimeSynchronizer([image_sub, info_sub], 10)
    ts.registerCallback(callback)
    rospy.spin()