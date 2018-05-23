import threading

import rospy
import cv2
import imutils
import colorsys

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

greenLower = (29, 86, 6)
greenUpper = (64, 255, 255)
ball_map = {"red_ball": ((157,0,0),(255,0,0)), "green_ball": (( 0,40,0),(0, 255,0)), "blue_ball": ((0,0,118),(0,0,255)) }

worker_threads = {}#[None] * len(ball_map)
results = {}

processedImage = CompressedImage()

def received_image_callback(value):

    conVerter = CvBridge()
    results.clear()
    image = conVerter.compressed_imgmsg_to_cv2(value, "bgr8")
    frame = imutils.resize(image, width=600)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    key = "green_ball"
    value = ball_map[key]
    color_range_lower = colorsys.rgb_to_hsv(value[0][0], value[0][1], value[0][2])
    color_range_upper = colorsys.rgb_to_hsv(value[1][0], value[1][1], value[1][2])
    findBalls(hsv, (greenLower, greenUpper), results, key)

    # Prepare a mutable list to store the results of each thread
    # for key,value in ball_map.iteritems():
    #     #Get HSV for color
    #     rospy.loginfo("Processing %s", key)
    #     color_range_lower = colorsys.rgb_to_hsv(value[0][0],value[0][1],value[0][2])
    #     color_range_upper = colorsys.rgb_to_hsv(value[1][0], value[1][1], value[1][2])
    #     worker_threads[key] = threading.Thread(name=key, target=findBalls, args=[hsv, (color_range_lower, color_range_upper), results, key])
    #     worker_threads[key].start()

    #for key, value in ball_map.iteritems():
    #    worker_threads[key].join()
    try:
        if len(results) > 0:
            cv2.circle(frame, (results[key][0],results[key][1]), results[key][2], ball_map[key][1],3)
    except KeyError:
        rospy.logerr("KeyError on %s", key);

    pub1.publish(conVerter.cv2_to_compressed_imgmsg(frame))

def findBalls(hsv, color_range, result, key):
    #rospy.loginfo("Looking for a %s ball", key)
    mask = cv2.inRange(hsv, color_range[0], color_range[1])
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)[-2]
    print "{}".format(cnts)
    if len(cnts)>0:
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        # only proceed if the radius meets a minimum size
        if radius > 10:
            rospy.loginfo("Found a %s ball", key)
            result[key] = (int(x), int(y), int(radius))

def Init():

    global pub1, pub2
    rospy.init_node('tbballtracker', anonymous=True)
    rospy.Subscriber('/rgb_image/compressed', CompressedImage, received_image_callback,None, 10)

    pub1 = rospy.Publisher('/processed_image/compressed', CompressedImage, queue_size=10)
    #pub2 = rospy.Publisher('/tbballtracker/image_result', Image, queue_size=10)
    rospy.spin();

#bT = BallTracker(ImgWidth, ImgHeight, "RED")

if __name__ == '__main__':
    try:
        Init()
    except rospy.ROSInterruptException:
        pass