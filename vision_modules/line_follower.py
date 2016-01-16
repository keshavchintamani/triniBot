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
c = max(contours, key=cv2.contourArea)
peri = cv2.arcLength(c, True)
approx = cv2.approxPolyDP(c, 0.05*peri, True)
rect = cv2.boundingRect(c)
print(rect)
cv2.drawContours(img, contours, -1, (255, 255, 2550, 4))
tl = (rect[0], rect[1])
print(tl)
br = (rect[2], rect[3])
print(br)
cv2.rectangle(img, tl, br, (255,255,255),8)
cv2.imshow("image", img)
cv2.waitKey(0)

#np.zeros((512,512,3), np.uint8)

# Draw a diagonal blue line with thickness of 5 px
#cv2.line(img,(0,0),(511,511),(255,0,0),5)


#Do some background subtraction
#fbg=cv2.createBackgroundSubtractorMOG2();
#img = fbg.apply(img)
#Now get the contour
#upper = np.array([0])
#lower = np.array([0])
#mask = cv2.inRange(img, lower, upper)

#(_, cnts, _) = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#c = max(cnts, key = cv2.contourArea)

#Approx the countour
#peri = cv2.arcLength(c,True)
#approx = cv2.approxPolyDP(c, 0.05*peri,True)

#cv2.drawContours(img, [approx], -1, (0.255,0), 4)
#cv2.imshow("image",img)
#cv2.waitKey(0)