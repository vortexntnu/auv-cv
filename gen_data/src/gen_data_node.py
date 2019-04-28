#! /usr/bin/python
# Copyright (c) 2015, Rethink Robotics, Inc.
# Edited 2019 by Thomas Hellum, Vortex NTNU

# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2

import numpy as np
import os
import math
import random
#from os.path import expanduser

# Paths and folder names 
save_dir = os.path.expanduser('~/Pictures/data')
train_dir = os.path.join(save_dir, "train")
validation_dir = os.path.join(save_dir, "validation")
test_dir = os.path.join(save_dir, "test")

# Max/min area of contour area for saving relevant images
min_area = 1
max_area = 10000

# Iterators for image names
i = 0
l = 0
j = 0

# Filter values (HSV)
H_lb = 160
H_ub = 20
S_lb = 105
S_ub = 255
V_lb = 105
V_ub = 255

# H_lb = 177
# H_ub = 8
# S_lb = 105
# S_ub = 255
# V_lb = 30
# V_ub = 255

# Instantiate CvBridge
bridge = CvBridge()

def image_callback(msg):
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError, e:
        print(e)
    else: # Extract and save relevant pictures

		# Init
		global i
		global j
		global l
		gate_found = False

		# Frames
		frame = cv2.resize(cv2_img, (331, 331))        	
		hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

		# Filter red
		hsv_lower_red1 = np.array([H_lb,S_lb,V_lb])
		hsv_upper_red1 = np.array([180,S_ub,V_ub])
		hsv_lower_red2 = np.array([0,S_lb,V_lb])
		hsv_upper_red2 = np.array([H_ub,S_ub,V_ub])

		# Create mask of filtered red
		maskr1 = cv2.inRange(hsv, hsv_lower_red1, hsv_upper_red1)
		maskr2 = cv2.inRange(hsv, hsv_lower_red2, hsv_upper_red2)
		mask = maskr1+maskr2

		# Check if object in frame
		_, cont, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
		contours_area = []
		for con in cont:
			area = cv2.contourArea(con)
			if min_area < area < max_area:
				contours_area.append(con)
				if len(contours_area) >= 2: # Only save if two poles
					gate_found = True

		# Save image
		if gate_found == True:
			j += 1
			if j == 30: # save every 10th frame
				j = 0

				print("Gate!", i)
				i += 1
				# Save the frame as train, validation or test set based on statistic properties
				if random.randint(0, 100) > 90:
					cv2.imwrite(test_dir + "/" + str(i) + ".png", frame)
				elif random.randint(0, 100) > 75:
					cv2.imwrite(validation_dir + "/" + str(i) + ".png", frame)
				else:
					cv2.imwrite(train_dir + "/" + str(i) + ".png", frame)

        # Display
		cv2.namedWindow('frame',cv2.WINDOW_NORMAL)
		cv2.resizeWindow('frame',600,400)
		cv2.imshow('frame', frame)

		cv2.namedWindow('mask',cv2.WINDOW_NORMAL)
		cv2.resizeWindow('mask',600,400)
		cv2.imshow('mask', mask)

		# Idk (but necessary)
		k = cv2.waitKey(5) & 0xFF
		l += 1
		if l == 100:
			l = 0
		if k == 27:
			return



def main():
    rospy.init_node('image_listener')

    # Create directories
    if not os.path.isdir(save_dir):
            os.makedirs(save_dir)

    if not os.path.isdir(train_dir):
            os.makedirs(train_dir)

    if not os.path.isdir(validation_dir):
            os.makedirs(validation_dir)

    if not os.path.isdir(test_dir):
            os.makedirs(test_dir)
    
    # Define your image topic
    image_topic = "/manta/manta/camerafront/camera_image" 		# "/manta/manta/cameraunder/camera_image" for simulator
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)
    # Spin until ctrl + c
    rospy.spin()


if __name__ == '__main__':
    main()
