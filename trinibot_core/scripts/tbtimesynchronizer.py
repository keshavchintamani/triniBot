#!/usr/bin/env python

import message_filters
from sensor_msgs.msg import Image, CameraInfo
import rospy


camerapub = rospy.Publisher('/image_synced', Image, queue_size=1000)
camerainfopub = rospy.Publisher('/camera_info_synced', CameraInfo, queue_size=1000)

def callback(image, camera_info):
    camerapub.publish(image)
    camerainfopub.publish(camera_info)
    rospy.loginfo("publishing synchronized image and camera info")

if __name__ == '__main__':

    rospy.init_node('tbmotioncontroller', anonymous=True)
    image_sub = message_filters.Subscriber('/image', Image)
    info_sub = message_filters.Subscriber('/camera_info', CameraInfo)

    ts = message_filters.TimeSynchronizer([image_sub, info_sub], 1000)
    #ts.connectInput()
    ts.registerCallback(callback)
    rospy.spin()