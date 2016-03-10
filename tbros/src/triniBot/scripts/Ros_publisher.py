#!/usr/bin/env python

import rospy
from tbVision import BallTracker
from trinibot_msgs.msg import ballcoords
ImgWidth = 640
ImgHeight = 480

def balltracker_publisher():

    pub = rospy.Publisher('coordinates', ballcoords, queue_size=10)
    rospy.init_node('balltracker_publisher', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        result = bT.computeSpheres()
        msg = ballcoords()
        if result == None:
            continue
        msg.color= "RED"
        msg.x = result[0]
        msg.y = result[1]
        msg.radius = result[2]
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

bT = BallTracker(ImgWidth, ImgHeight, "RED")

if __name__ == '__main__':
    try:
        balltracker_publisher()
    except rospy.ROSInterruptException:
        pass