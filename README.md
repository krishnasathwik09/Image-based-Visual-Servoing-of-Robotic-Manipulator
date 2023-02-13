# Image-based-Visual-Servoing-of-Robotic-Manipulator
![ezgif com-crop](https://user-images.githubusercontent.com/93411202/218357364-1d6df1db-f7ac-4b96-8ff1-27d1ec651178.gif)
![Left](https://user-images.githubusercontent.com/93411202/218358204-ab195c04-c035-4760-9ff1-603e3e6b7f0c.gif)
![Right](https://user-images.githubusercontent.com/93411202/218358218-2c0ea29a-a5be-4567-811d-b00b7dc70fac.gif)


This package simulates a IBVS(Image based Visual Servoing) - Eye to Hand configuration for 2 link Planar Manipulator.

Gazebo Setup:
![Setup](https://user-images.githubusercontent.com/93411202/218359163-888c8f50-0e26-4a34-9983-274b72bae842.png)




**Feature Detection Node.** 

Implemented a ROS node to detect End effector Feature point using Color segmentation method in OPENCV.

**Proportional IBVS Node.**

-> Eye to Hand Configuration

-> Image Jacobian with constant Depth.

-> Robot Jacobian w.r.t Camera Frame

-> Proportional Controller

## Dependecies:
This repository has been developed and tested in Ubuntu 20.04 and ROS Noetic.

## Setup and Run

i) Launch the nodes using below command.

  `roslaunch planar_robot_vs planarbot_vs.launch`
