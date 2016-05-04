#!/usr/bin/env python

import threading
import rospy
from tbVision import BallTracker
import roslib
import time as Time

ImgWidth = 640
ImgHeight = 480

from balltracker.msg import ballcoords
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def callback(value):

    worker_thread = threading.Thread(name = "ballfinder_worker", target = processFrame, args = [value])
    worker_thread.start()
    worker_thread.join()

def processFrame(image):

    global pub1, pub2
    conVerter = CvBridge()

    #Convert from a ROS image to a CV image
    try:
        image = conVerter.imgmsg_to_cv2(image, "bgr8")
    except CvBridgeError as e:
        print (e)

    result = bT.computeSpheres(image)
    msgBallCoords = ballcoords()
    msgModImage = Image()
    if result == None:
        return
    msgBallCoords.color= "RED"
    msgBallCoords.x = result[0]
    msgBallCoords.y = result[1]
    msgBallCoords.radius = result[2]

    # convert from a CV image to a ROS image

    resImage = result[3]

    #resImage = conVerter.cv2_to_imgmsg(resImage , "bgr8")
    msgModImage = conVerter.cv2_to_imgmsg(resImage , "bgr8")

    rospy.loginfo(msgBallCoords)
    pub1.publish(msgBallCoords)
    pub2.publish(msgModImage)

def Init():

    global pub1, pub2
    rospy.init_node('tbballtracker', anonymous=True)
    rospy.Subscriber('/cv_camera/image_raw', Image , callback)
    pub1 = rospy.Publisher('/tbballtracker/coordinates', ballcoords, queue_size=10)
    pub2 = rospy.Publisher('/tbballtracker/image_result', Image, queue_size=10)
    while not rospy.is_shutdown():
        rospy.loginfo("Doing something")
        Time.sleep(2)

bT = BallTracker(ImgWidth, ImgHeight, "RED")

if __name__ == '__main__':
    try:
        Init()
    except rospy.ROSInterruptException:
        pass