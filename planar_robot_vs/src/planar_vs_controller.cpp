#include "ros/ros.h"
#include <math.h>
#include <iostream>
#include <vector>
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include <eigen3/Eigen/Dense>
#include "sensor_msgs/JointState.h"
#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>
#include <cstdlib>

/**** Formualtion 
 *  Camera Intrinsic Matrix K -(pixel frame = K * Camera frame)
 *   [Z*U Z*V Z] =  K * [Xc Yc Z]
 * 
 *  K =[f 0 C;
 *      0 f C;
 *      0 0 1]
 *   
 *   U_pixel = (f/Z) * Xc  +  C
 *   V_pixel = (f/Z) * Yc  +  C
 * 
 *  Planar (2D) => constant Depth => Z_dot = 0
 * 
 *  Pixel velocities = [u_dot ,v_dot]
 *  u_dot = (f/Z) * Xc_dot -----1
 *  v_dot = (f/Z) * Yc_dot -----2
 *  
 *  Xc_dot = -V_x  - (w_y*Z) + (w_z*Y) ----> 3
 *  Yc_dot = -V_y  - (w_z*X) + (w_x*Z) ----> 4
 *  
 * Substitue 3 & 4 in 1 & 2 
 * 
 * [u_dot v_dot]  = J_img * ee_vel
 * 
 *  J_img(2*6) << -(f/Z), 0, 0, 0,-f, (V_pixel - C),
            0, -(f/Z) , 0 , f ,0 , (C - U_pixel);
 * 
 * ****Robot Jacobian is derived w.r.t Camera center Frame ***
 * 
 * J_robot << ((l1*cos(theta1))+(l2*cos(theta1+theta2))), ((l2*cos(theta1+theta2))),
              ((l1*sin(theta1))+(l2*sin(theta1+theta2))) ,((l2*sin(theta1+theta2)));

 * Proportional Controller Gains 
 * lamda = [lamda_1 , lamda_2]
 * 
 *  Proportional Controller Law - Pixel Position error is proportional to pixel velocities
 *
 * Combined_Jacobian(2*2) = J_image(2*6) * J_image(6*2)
 * 
 * joint_vel = lamda * inverse(Combined_Jacobian) * error_vec;
 * 
 * ****/

float ee_pos_x;  // Feature cordinates in Pixel frame
float ee_pos_y;  // Feature cordinates in Image frame
float theta1;    // Joint -1 angle
float theta2;    // Joint -2 angle

/***Callback for End Effector Marker Point Topic***/
void ee_feature_callback(const std_msgs::Float64MultiArray &msg){
    ee_pos_x = msg.data.at(0);
    ee_pos_y = msg.data.at(1);
}

/***Callback for Joint angles Topic***/
void joint_states_callback(const sensor_msgs::JointState msg){
     theta1 = msg.position[0];
     theta2 = msg.position[1];
}


int main(int argc, char **argv){

    // ROS initialization
    ros::init(argc, argv, "vs_controller");
    ros::NodeHandle n;
    
    ros::Rate rateController = ros::Rate(10);

    // Initializing ROS publishers
    ros::Publisher j1_pub = n.advertise<std_msgs::Float64>("/planarbot/joint1_velocity_controller/command",1);
    ros::Publisher j2_pub = n.advertise<std_msgs::Float64>("/planarbot/joint2_velocity_controller/command",1);
    ros::Publisher err_pub = n.advertise<std_msgs::Float64MultiArray>("servoing_error", 1);

    // Subscriber Node for End effector feature pose
    ros::Subscriber ee_feature = n.subscribe("ee_feature_pose",1,ee_feature_callback);
    ros::Subscriber joints_poses = n.subscribe("/planarbot/joint_states",1,joint_states_callback);
    ros::Duration(15).sleep(); 
    // Declarations
    std_msgs::Float64 j1_vel; // joint1 velocity
    std_msgs::Float64 j2_vel; // joint2 velocity
    std_msgs::Float64MultiArray err_msg; //Error msg topic
    Eigen::MatrixXf k_intrinsic(3,3);
    Eigen::Vector2f error_vec;
    Eigen::Vector2f joint_vel;
    Eigen::MatrixXf J_robot(6,2); // Robot Jacobian
    Eigen::MatrixXf J_img(2,6); // Image Jacobian
    Eigen::MatrixXf J_combined(2,2); // Image Jacobian * Robot Jacobian

    float err;  // Norm of Pixel(x,y) Errors
    float ee_img_x; // Feature cordinates in Image frame
    float ee_img_y; // Feature cordinates in Image frame

    float l1 = 0.5; //link-1 length
    float l2 = 0.5; //link-2 length

    /**Gazebo Camera Parameters from Intrinsic Matrix**/
    float focus = 178.76; //Focus from Gazebo Camera info
    float Z = 1.83; //depth-2
    float focus_const = focus/Z; // 178.76/2
    float c_intrinsic = 150.5; //principle point centre offset

    /*** Extract parameters from config.yaml file ***/
    float goal_pos_x; //Goal Feature positions  150,136
    float goal_pos_y; // 130,118
    float thres_val; //Control loop Termination Threshold
    Eigen::Vector2f lamda; // Proportional Gain //1.5
    n.getParam("planarbot/control/goal_pos_x",goal_pos_x);
    n.getParam("planarbot/control/goal_pos_y",goal_pos_y);
    n.getParam("planarbot/control/lambda_1",lamda[0]);
    n.getParam("planarbot/control/lambda_2",lamda[1]);
    n.getParam("planarbot/control/thresh",thres_val);

    /* Initial Error */
    error_vec[0] = goal_pos_x - ee_pos_x;
    error_vec[1] = goal_pos_y - ee_pos_y;
    err = sqrt((error_vec[0]*error_vec[0]) + (error_vec[1]*error_vec[1]));

    /** Initialize Zero Joint Velcoities **/
    j1_vel.data = 0;
    j2_vel.data = 0;
    j1_pub.publish(j1_vel);
    j2_pub.publish(j2_vel);

    std::cout<<" Servoing Started "<<std::endl;
    /********** Control loop **************/
    while(err > thres_val){
   
        // update error 
        error_vec[0] =  (goal_pos_x - ee_pos_x);
        error_vec[1] = (goal_pos_y - ee_pos_y);
        err = sqrt((error_vec[0]*error_vec[0]) + (error_vec[1]*error_vec[1]));

        if((error_vec[0] != goal_pos_x) &&  (error_vec[1] != goal_pos_y))
        {
            // Publish current error to plot data
            err_msg.data.clear();
            err_msg.data.push_back(err);
            err_msg.data.push_back(error_vec[1]);
            err_pub.publish(err_msg);

            /**Robot Jacobian**/
            J_robot << ((l1*cos(theta1))+(l2*cos(theta1+theta2))), ((l2*cos(theta1+theta2))),
            ((l1*sin(theta1))+(l2*sin(theta1+theta2))) ,((l2*sin(theta1+theta2))),
            0,0,
            0,0,
            0,0,
            1,1;
            /**Image Jacobian- Constant Depth**/
            J_img << -focus_const, 0, 0, 0,-focus, (ee_pos_y - c_intrinsic),
            0, -focus_const , 0 , focus ,0 , (c_intrinsic - ee_pos_x);

            J_combined = J_img * J_robot;
            /** Proportional Control Law **/
            joint_vel = lamda.cwiseProduct(J_combined.inverse() * error_vec);

            /** limit joint velocities **/
            if(abs(joint_vel[0]) > 5){
                if(joint_vel[0]>0){
                    joint_vel[0] = 5;
                }
                else{
                    joint_vel[0] = -5; 
                }
            }
            if(abs(joint_vel[1]) > 5){
                if(joint_vel[1]>0){
                    joint_vel[1] = 5;
                }
                else{
                    joint_vel[1] = -5; 
                }
            }

            // Publish Joint Velocities
            j1_vel.data = joint_vel[0];
            j2_vel.data = joint_vel[1];
            j1_pub.publish(j1_vel);
            j2_pub.publish(j2_vel); 
        }
        ros::spinOnce();
        rateController.sleep();
    
    }

    j1_vel.data = 0.0;
    j2_vel.data = 0.0;
    j1_pub.publish(j1_vel);
    j2_pub.publish(j2_vel);
    std::cout<<" Servoing Completed "<<std::endl;
    ros::spin();
    return 0;
}