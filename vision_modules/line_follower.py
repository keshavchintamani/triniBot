import numpy as np
import cv2

# Create a black image
img = cv2.imread("line.jpg")

thresh = 100
max_thres = 255

img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
img = cv2.blur(img,(3,3))
img = cv2.Canny(img, thresh,thresh*3)
(_,contours,_) = cv2.findContours(img,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

print "we found %d contours" % len(contours)
c = contours[0]
epsilon = 0.1*cv2.arcLength(c, True)
approx = cv2.approxPolyDP(c, epsilon, True)
rect = cv2.minAreaRect(c)
print rect[0]
box = cv2.boxPoints(rect)
print box
box = np.int0(box)

# Find moments
M = cv2.moments(box)
cx = int(M['m10']/M['m00'])
cy = int(M['m01']/M['m00'])
print cx, cy
cv2.circle(img, (cx,cy), 20, (255), 3)
cv2.drawContours(img, [box], -1, (255, 255, 255, 4))
cv2.namedWindow('image')
cv2.imshow("image", img)
cv2.waitKey(0)
