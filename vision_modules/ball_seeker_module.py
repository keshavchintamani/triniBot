from collections import deque
import numpy as np
import argparse
import imutils
import cv2
import threading
import Queue as queue

class BallTracker(threading.Thread):

    def __init__(self, q, width, height):
        threading.Thread.__init__(self)
        # Create an affine transform
        self.W = width 
        self.H = height
        self.C = np.matrix([[0],[0],[0],[0]])
        self.qUe = q;
        
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
        self.qUe.push((xp,yp))
        return xp, yp

    def startBallTracker(self):
        colorLower = (0, 100, 100)
        colorUpper = (179, 255, 255)
        #pts = deque(maxlen=args["buffer"])
        camera = cv2.VideoCapture(0)
        while True:
            # grab the current frame
            (grabbed, frame) = camera.read()
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
                #print "Center={} {}".format(center[0], center[1])
                # only proceed if the radius meets a minimum size
                if radius > 10:
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    cv2.circle(frame, (int(x), int(y)), int(radius),
                       (0, 255, 255), 2)
                    cv2.circle(frame, center, 5, (0, 0, 255), -1)
                    cart_coords = self.transform_coordinates(center[0], center[1])
                    print "{}".format(cart_coords)

            # show the frame to our screen
            cv2.imshow("Frame", frame)
            key = cv2.waitKey(1) & 0xFF
            # if the 'q' key is pressed, stop the loop
            if key == ord("q"):
                break

    def cleanUp(self):
        # cleanup the camera and close any open windows
        camera.release()
        cv2.destroyAllWindows()

    def run(self):
        self.startBallTracker()


if __name__ == "__main__":

    q = queue()
    bt = BallTracker(q, 640, 480)
    bt.start()
    
    

    
