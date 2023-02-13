# Image-based-Visual-Servoing-of-Robotic-Manipulator
![ezgif com-crop](https://user-images.githubusercontent.com/93411202/218357364-1d6df1db-f7ac-4b96-8ff1-27d1ec651178.gif)
![Right](https://user-images.githubusercontent.com/93411202/218357744-66746286-c29d-43b4-99fb-abe6c1306db3.gif)




This package simulates a IBVS(Image based Visual Servoing) - Eye to Hand configuration for 2 link Planar Manipulator.

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
