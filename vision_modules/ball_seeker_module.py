from collections import deque
import numpy as np
import argparse
import imutils
import cv2

import threading
from Queue import Queue
import time as Time

BUFFER = 32
class BallTracker(threading.Thread):

    #Pass a callback cb with image coordinates w.r.t. image center x, y and radius
    #e.g f(x,y,radius)
    def __init__(self, cb, width, height, ballColor):
        self.supportedColors = {'RED', 'GREEN', 'BLUE'}
        if ballColor in self.supportedColors:
            self.Color = ballColor
        else:
            print "Sorry Invalid color "
            return
        self.W = width 
        self.H = height
        self.C = np.matrix([[0],[0],[0],[0]])
        self.callBack = cb;
        self.isRunning = False;
        self.setupTransforms()
        print "Initializing camera"
        self.camera = cv2.VideoCapture(1)
        print "Success initializing camera; starting process"
        
    def setupTransforms(self):
        P1 = (0, 0)
        P2 = (self.W,self.H)
        p1 = (-1, 1)
        p2 = (1, -1)
        M = np.matrix([[P1[0], P1[1], 1, 0], [-P1[1], P1[0], 0, 1], [P2[0], P2[1], 1, 0], [-P2[1], P2[0], 0, 1]])
        Minv = M.getI()
        tC = np.matrix([[p1[0]], [p1[1]], [p2[0]], [p2[1]]])
        self.C = Minv * tC

    def transform_coordinates(self, x, y):
        xp = self.C[0] * x + self.C[1] * y + self.C[2]
        yp = self.C[1] * x -self.C[0] * y + self.C[3]
        return float(xp), float(yp)

    def startBallTracker(self):
        self.isRunning=True
        #t = threading.Thread(target = self.computeSpheres)
        #t.start()
        self.computeSpheres()
        Time.sleep(0.01)

    def stopBallTracker(self):
        self.isRunning=False
        Time.sleep(0.01)
        self.cleanUp()

    def computeSpheres(self):

        #Get the HSV color based on user request
        #Default is blue
        if self.Color == "GREEN":
            colorLower = (29, 86, 6)
            colorUpper = (64, 255, 255)
        elif self.Color == "RED":
            colorLower = (0, 100, 100)
            colorUpper = (179, 255, 255)
        elif self.Color == "BLUE":
            colorLower = (210, 100, 100 )
            colorUpper = (255, 0, 0)

        pts = deque(maxlen=BUFFER)

        while True:
            # grab the current frame
            (grabbed, frame) = self.camera.read()
            # if we are viewing a video and we did not grab a frame,
            # then we have reached the end of the video
            #if args.get("video") and not grabbed:
            #    break
            # resize the frame, blur it, and convert it to the HSV
            # color space
            frame = imutils.resize(frame, self.W, self.H)
            frame = cv2.flip(frame, 1)
            blurred = cv2.GaussianBlur(frame, (11, 11), 0)
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            # construct a mask for the color "green", then perform
            # a series of dilations and erosions to remove any small
            # blobs left in the mask
            mask = cv2.inRange(hsv, colorLower, colorUpper)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)
            # find contours in the mask and initialize the current
            # (x, y) center of the ball
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)[-2]
            center = None

            # only proceed if at least one contour was found
            if len(cnts) > 0:
                # find the largest contour in the mask, then use
                # it to compute the minimum enclosing circle and
                # centroid
                c = max(cnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                if radius > 10:
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    cv2.circle(frame, (int(x), int(y)), int(radius),
                       (0, 255, 255), 2)
                    cv2.circle(frame, center, 5, (0, 0, 255), -1)
                    pts.appendleft(radius)
                    radSum=pts[0]
                    for i in xrange(1, len(pts)):
                        # if either of the tracked points are None, ignore
                        # them
                        if pts[i - 1] is None or pts[i] is None:
                            continue
                        # otherwise, compute the thickness of the line and
                        # draw the connecting lines
                        #thickness = int(np.sqrt(BUFFER / float(i + 1)) * 2.5)
                        #cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)
                        radSum += pts[i]

                    ballX, ballY = self.transform_coordinates(center[0], center[1])
                    self.callBack(ballX, ballY, int(radSum/len(pts)))
                else:
                    self.callBack(None, None, None)

            # show the frame to our screen
            cv2.imshow("Frame", frame)
            key = cv2.waitKey(1) & 0xFF
            # if the 'q' key is pressed, stop the loop
            if key == ord("q"):
                break
            #Time.sleep(1)

    def cleanUp(self):
        # cleanup the camera and close any open windows
        self.camera.release()
        cv2.destroyAllWindows()

def myBallTrackerCallBack(x,y, radius):
    print "Recieved:", x, y, radius


if __name__ == "__main__":

    q = Queue()
    bt = BallTracker(myBallTrackerCallBack, 320, 240, "RED")
    bt.startBallTracker()
    #while True:
        #print "getting coords"
        #res = q.get()
        #print res
        #cv2.imshow("Image", res["Frame"])
        #print x, y
    #   Time.sleep(0.5)
    #bt.stopBallTracker()
    

    
