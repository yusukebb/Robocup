#!/usr/bin/env python3

# Importing the necessary libraries
import rospy
import sys
import cv2
import numpy as np

from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

################################################################################################

class track_red_ball:

    def __init__(self):

        self.rate = rospy.Rate(1)
        self.bridge = CvBridge()

        # Publisher to move robot
        self.vel_pub = rospy.Publisher('/mark_fbj/diff_drive_controller/cmd_vel', Twist, queue_size=10)
        self.twist = Twist()

        # Image subscriber to read data from camera
        img_sub = rospy.Subscriber('/camera/image_raw', Image, self.process_img) 


    def process_img(self, data):
        try:
            # Converting ROS image topic to CV format
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")

            # Extracting image dimensions
            h, w, d = img.shape

            # Setting lower and upper limit for red color
            redLower = np.array([5, 50, 50])
            redUpper = np.array([15 , 255, 255])

            # Adding a mask to extract only the red ball
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, redLower, redUpper)

            # Detecting edges by adding Gaussian Blur and then using canny edge detection
            gray_img = cv2.GaussianBlur(mask, (5, 5), 0)
            edges = cv2.Canny(gray_img, 35, 125)

            # Storing the largest contour
            contours, hierarchy = cv2.findContours(edges.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

            # Calculating centroid of the circle using moment method
            M = cv2.moments(mask)
            try:
                self.cx = int(M['m10']//M['m00'])
                self.cy = int(M['m01']//M['m00'])

            # Defining the case in which m00 = 0
            except ZeroDivisionError:
                self.cx, self.cy = 0, 0

            # Drawing a circle around the object
            # and drawing centroid for visualization
            moij = str(self.cx) + str(self.cy)
            cv2.circle(img, (self.cx, self.cy), 20, (0, 100, 100), -1)
            cv2.putText(img, moij, (self.cx - 25, self.cy - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255))

            # Calculating x coordinate from center of axis
            xx = (930 - self.cx)

            # Storing intermediate xx values here
            prev_xx = 0
            rotation = 0

            # If the centroid is not visible in the image, this condition is satisfied
            if (self.cx == 0 and self.cy == 0):
                # Rotating the robot if ball is not visible in x range
                if prev_xx > 0: 
                    self.twist.angular.z = 0.05
                    self.twist.linear.x = 0
                    self.vel_pub.publish(self.twist)
                    rotation = 0.05
                    rospy.loginfo(cy)


                # Rotating the robot in opposite direction if prev_xx is in the opposite
                # direction
                else:
                    self.twist.angular.z = -0.05
                    self.twist.linear.x = 0
                    self.vel_pub.publish(self.twist)
                    rotation = -0.05

            # This condition is executed when centroid is visible 
            else:
                # Setting an arbitrary value less than which we move the robot towards the centroid
                if self.cy <= 700:
                    # Checking if the absolute distance is in range and then rotating 
                    # robot in a tuned angular velocity
                    if xx >= 20 and xx <= -20:
                        # Setting a proportional angular velocity
                        self.twist.angular.z = rotation
                        self.twist.linear.x = 0
                        self.vel_pub.publish(self.twist)

                        prev_xx = xx

                    # If ball is in x range, and in front of the robot
                    # move in a straight line
                    elif xx < 20 and xx > -20:
                        self.twist.angular.z = 0
                        self.twist.linear.x = 0.8
                        self.vel_pub.publish(self.twist)

                        prev_xx = xx



                    # Setting an arbitrary value less than which we move the robot towards the centroid
                if xx >= 20 and xx <= -20:
                    self.twist.angular.z = rotation
                    self.twist.linear.x = 0
                    self.vel_pub.publish(self.twist)

                    prev_xx = xx

                elif xx < 20 and xx > -20:

                    if self.cy < 700:
                     
                    self.twist.angular.z = 0
                    self.twist.linear.x = 0.8
                    self.vel_pub.publish(self.twist)

                    prev_xx = xx


                    # Checking if the absolute distance is in range and then rotating 
                    # robot in a tuned angular velocity
                    if xx >= 20 and xx <= -20:
                        # Setting a proportional angular velocity
                        self.twist.angular.z = rotation
                        self.twist.linear.x = 0
                        self.vel_pub.publish(self.twist)

                        prev_xx = xx

                    # If ball is in x range, and in front of the robot
                    # move in a straight line
                    elif xx < 20 and xx > -20:
                        self.twist.angular.z = 0
                        self.twist.linear.x = 0.8
                        self.vel_pub.publish(self.twist)

                        prev_xx = xx




                # This condition is satisfied if the ball is too close to the robot
                elif self.cy > 700:
                    # Stopping the robot
                    self.twist.angular.z = 0
                    self.twist.angular.x = 0
                    self.vel_pub.publish(self.twist)

            # Showing the live camera view from the robot in openCV window
            img_size_devide = 3
            resize_img = cv2.resize(img, (int(w/img_size_devide), int(h/img_size_devide)))
            cv2.imshow('window', resize_img)
            cv2.waitKey(5)

        except CvBridgeError as e:
            print(e)

################################################################################################

# Initializing the ball tracking and following node
rospy.init_node('track_red_ball', anonymous=True)

trb = track_red_ball()

rospy.spin()

