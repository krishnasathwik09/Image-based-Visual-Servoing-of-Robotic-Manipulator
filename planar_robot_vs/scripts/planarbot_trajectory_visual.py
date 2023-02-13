#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
import glob
import os
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Int32

bridge = CvBridge()




r = 20

traj_pts_x = []
traj_pts_y = []



def eePosCallback(msg):
    global traj_pts_x, traj_pts_y, status
    traj_pts_x.append(int(msg.data[0]))
    traj_pts_y.append(int(msg.data[1]))


def visCallback(msg):

    global bridge, goalX, goalY, img_pub

    try:
        cv_img = bridge.imgmsg_to_cv2(msg,"bgr8")
    except CvBridgeError as e:
        print(e)


    cv_img = cv2.circle(cv_img,(goalX, goalY), radius = 3, color=(0,220,0), thickness=2)

    for i in range(len(traj_pts_x) - 1):
        start_pt = (traj_pts_x[i], traj_pts_y[i])
        end_pt = (traj_pts_x[i+1], traj_pts_y[i+1])
        cv_img = cv2.line(cv_img, start_pt, end_pt,(0,0,240), 2)
    
    # convert cv image to ROS msg
    try:
        ros_img = bridge.cv2_to_imgmsg(cv_img,"bgr8")
    except CvBridgeError as e:
        print(e)

    img_pub.publish(ros_img)




def main(args):
    global goalX, goalY, img_pub
    
    # Initialize ROS
    rospy.init_node('visualizer')
    
    # Goal Position
    goalX = rospy.get_param("planarbot/control/goal_pos_x")
    goalY = rospy.get_param("planarbot/control/goal_pos_y")

    img_pub = rospy.Publisher("planarbot/vis", Image, queue_size=1)

    # Initialize subscribers
    img_sub = rospy.Subscriber("planarbot/camera1/image_raw", Image, visCallback, queue_size = 1)
    ee_sub = rospy.Subscriber("/ee_feature_pose", Float64MultiArray, eePosCallback, queue_size=1)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == "__main__":
    main(sys.argv)
