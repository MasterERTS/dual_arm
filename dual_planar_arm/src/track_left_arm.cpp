//Cpp
#include <sstream>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <math.h>
//#include <cmath.h>

//ROS
#include "ros/ros.h"

// My Packages
#include <dual_planar_arm_msgs/DualArmIK.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>

geometry_msgs::PoseStamped goalPose;

void rightArmGoalCallback(geometry_msgs::PoseStamped receivedGoalPose) {
    goalPose = receivedGoalPose;
}

int main (int argc, char** argv)
{

	//ROS Initialization
    ros::init(argc, argv, "left_arm_tracker");

    // Define your node handles: YOU NEED AT LEAST ONE !
    ros::NodeHandle nh_glob, nh_loc("~") ;

    // Declare you publishers and service servers
    ros::ServiceClient client = nh_glob.serviceClient<dual_planar_arm_msgs::DualArmIK>("/right_arm_IK_service");
    ros::Subscriber subGoalPose = nh_glob.subscribe<geometry_msgs::PoseStamped>("/goal_pos", 1, rightArmGoalCallback);

    ros::Publisher pubRightArm = nh_glob.advertise<sensor_msgs::JointState>("/joint_command", 1);

    // messages Initialization
    dual_planar_arm_msgs::DualArmIK service_msg;

    ros::Rate rate(100);   // Or other rate.
    while (ros::ok()) {
        ros::spinOnce();

        // Everything is done in the callback function. 
        // This loop only defines the frequency at which incoming messages are attended to.
        if (client.exists() && client.isValid()) {
            service_msg.request.goal.header.frame_id = "r_arm_base";
            service_msg.request.goal.point.x = goalPose.pose.position.x;
            service_msg.request.goal.point.y = goalPose.pose.position.y;
            bool successful = client.call(service_msg);
            if (!successful) {
                ROS_INFO("Request call was unsuccessfull...");
            } else {
                std::size_t n_solutions = service_msg.response.solutions.size();
                if (n_solutions > 0) {
                    sensor_msgs::JointState r_joints_solutions = service_msg.response.solutions[0];
                    pubRightArm.publish(r_joints_solutions);
                }
            }
        } else {
            ROS_INFO("Service Client not ready or doesn't exist..");
        }
        rate.sleep();
    }
}
