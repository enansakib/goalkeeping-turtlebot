#!/usr/bin/env python
import cv2
import rospy
from std_msgs.msg import Float32
import imutils
from sensoRmsgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import math

# global variables defined to eliminate some inconsistent detections
frame_no=0
X = None
Y = None
R = None

def show_image(img):
    cv2.imshow("Image Window", img)
    cv2.waitKey(1)

def callback(img_msg):
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    except CvBridgeError as e:
        print "CvBridge Error: {}".format(e)
    # resizing the image, finding the mask for the ball, calculating valid contours
    frame = imutils.resize(cv_image, width=600)
    hsv = cv2.cvtColor(frame, cv2.COLORBGR2HSV)
    mask = cv2.inRange(hsv, greenLower, greenUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    cnts = cv2.findContours(mask.copy(), cv2.RETREXTERNAL,cv2.CHAIN_APPROXSIMPLE)[-2]
    center = None
    # If countours are found (at least one) then find the largest one and estimate
    # the minimum enclosing circle of the largest contour
    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c) # Centroid of the contour = (x, y)
        # Consistency check of the detection
        global frame_no, X, Y, R
        # If the distance between old and new centroids is high or, current radius changes abruptly
        # or, the detection is not consistent for at least 3 frames then we consider the detection
        # as invalid and publish a number 700 to keep track of this
        if X == None and Y == None and radius:
            X = x
            Y = y
            R = radius
            frame_no = 1
            show_image(frame)
            pub.publish(700)
            return

        elif math.sqrt((X-x)*(X-x)+(Y-y)*(Y-y)) > 20 or radius > 1.2*R or radius < 0.8*R:
            X = None
            Y = None
            R = None
            frame_no = 0
            show_image(frame)
            pub.publish(700)
            return

        elif (math.sqrt((X-x)*(X-x)+(Y-y)*(Y-y)) < 20 or (radius < 1.2*R and radius > 0.8*R)) and frame_no < 3:
            X = x
            Y = y
            R = radius
            frame_no += 1
            show_image(frame)
            pub.publish(700)
            return

        elif (math.sqrt((X-x)*(X-x)+(Y-y)*(Y-y)) < 20 or (radius < 1.2*R and radius > 0.8*R)) and frame_no > 3:
            X = x
            Y = y
            R = radius
        # Calculating Moments from the largest contour
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        # publish x coordinate of the center
        pub.publish(center[0])
        rate.sleep()
        # This is just to visualize the tracking
        if radius > 10:
            cv2.circle(frame, (int(x), int(y)), int(radius),(0, 0, 255), 2)
            cv2.circle(frame, center, 5, (0, 255, 255), -1)
    else:
        # If no contour is found, i.e., no ball or ball went out of the frame keep track of this situation by
        # publishing 700
        pub.publish(700)

    show_image(frame)


rospy.init_node('ball_tracking')
bridge = CvBridge()
pub = rospy.Publisher('coordinates', Float32, queue_size=10)
rate = rospy.Rate(50)
#tennis ball
greenLower = (29, 86, 6)
greenUpper = (64, 255, 255)
sub = rospy.Subscriber('/zed/zed_node/rgb_raw/image_raw_color', Image, callback)
# Loops infinitely until someone stops the program execution
rospy.spin()
