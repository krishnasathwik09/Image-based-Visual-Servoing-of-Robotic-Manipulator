#!/usr/bin/env python3

import rospy
import sys
import cv2
from std_msgs.msg import String
from std_msgs.msg import Int64, Float64MultiArray
from sensor_msgs.msg import Image
import matplotlib.pyplot as plt
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import sys, os
# import NotCvBridge as bridge
bridge = CvBridge()

pubCenter = rospy.Publisher('ee_feature_pose', Float64MultiArray, queue_size=1)


# Define a callback for the Image message
def image_callback(ros_image):
	#print('got an image')
	global bridge
	#  r = rospy.get_param("vsbot/vs_baseline/pub_rate")
	r = rospy.get_param("planarbot/estimation/rate")
	rate = rospy.Rate(r)
	
	try:
		img = bridge.imgmsg_to_cv2(ros_image, "bgr8")

	except CvBridgeError as e:
		print(e) 


	hsvimg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

	#taking binary masks of the end effector marker colored blue	  	  
	blue_lower = np.array([94, 80, 2], np.uint8) 
	blue_upper = np.array([120, 255, 255], np.uint8) 
	blue_mask = cv2.inRange(hsvimg, blue_lower, blue_upper)
	
	kernel = np.ones((5, 5), "uint8")
	blue_mask_contour = cv2.dilate(blue_mask, kernel)
	res_blue = cv2.bitwise_and(img, img, mask = blue_mask)
	
	#taking binary mask of white pixels to complete binary convertion of the blue segment
	white_lower = np.array([0, 0, 200], np.uint8)
	white_upper = np.array([145, 60, 255], np.uint8)   
	white_mask = cv2.inRange(hsvimg, white_lower, white_upper)
	
	orange_lower = np.array([10, 100, 20], np.uint8)
	orange_upper = np.array([25, 255, 255], np.uint8)
	orange_mask = cv2.inRange(hsvimg, orange_lower, orange_upper)
	
	# Blue segment converted to binary
	mask1 = blue_mask + white_mask
	
	#mask2 = orange_mask + white_mask
	
	#First way of finding center of contour
	pixels = np.argwhere(mask1 > 0)
	
	#print(pixels)
	# find the first and last x components of binary pixels
	x1 = pixels[0][1]
	x2 = pixels[-1][1]
	
	#find the first and last y components of the binary pixels
	y1 = pixels[0][0]
	y2 = pixels[-1][0] 
	 
	
	#second way of finding center of marker
	contours, hierarchy = cv2.findContours(blue_mask_contour, 
	                                   cv2.RETR_TREE, 
	                                   cv2.CHAIN_APPROX_SIMPLE) 
	  

	for contour in contours:
		M = cv2.moments(contour)
		cX = int(M["m10"] / M["m00"])
		cY = int(M["m01"] / M["m00"])


	cv2.circle(img,(cX, cY), radius = 1, color=(0,0,0), thickness=2)

	cv2.imwrite('image.jpg', img)
	
	blue_center = Float64MultiArray()
	blue_center.data = np.array([cX,cY])
	
	pubCenter.publish(blue_center)
            
	                 
	  

def main(args):
  rospy.init_node('ee_detection', anonymous=True)  
  image_sub = rospy.Subscriber("planarbot/camera1/image_raw",Image, image_callback, queue_size=1)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)



    
