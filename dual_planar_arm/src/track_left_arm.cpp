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
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>

int main (int argc, char** argv)
{

	//ROS Initialization
    ros::init(argc, argv, "left_arm_tracker");

    // Define your node handles: YOU NEED AT LEAST ONE !
    ros::NodeHandle nh_glob, nh_loc("~") ;

    // Declare you publishers and service servers
    ros::ServiceClient client = nh_glob.serviceClient<dual_planar_arm_msgs::DualArmIK>("/right_arm_IK_service");

    ros::Publisher pubRightArm = nh_glob.advertise<sensor_msgs::JointState>("/joint_command", 1);
 
    // messages Initialization
    dual_planar_arm_msgs::DualArmIK service_msg;

    tf::TransformListener listener;
    tf::StampedTransform transform;

    listener.waitForTransform("/r_arm_base", "r_arm_goal", 
                                ros::Time(0), ros::Duration(5.0), ros::Duration(2));

    ros::Rate rate(100);   // Or other rate.
    while (ros::ok()) {
        ros::spinOnce();

        try{
        listener.lookupTransform("/r_arm_base", "r_arm_goal",
                                ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        continue ;
        }

        geometry_msgs::PoseStamped goalPose;
        
        // Enter Goal Position from Transform 
        goalPose.header.frame_id = "r_arm_goal";
        goalPose.pose.position.x = transform.getOrigin().x();
        goalPose.pose.position.y = transform.getOrigin().y();
        goalPose.pose.position.z = transform.getOrigin().z();
        goalPose.pose.orientation.x = transform.getRotation().x();
        goalPose.pose.orientation.y = transform.getRotation().y();
        goalPose.pose.orientation.z = transform.getRotation().z();
        goalPose.pose.orientation.w = transform.getRotation().w();

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
