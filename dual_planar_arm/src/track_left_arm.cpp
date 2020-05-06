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


int main (int argc, char** argv)
{

	//ROS Initialization
    ros::init(argc, argv, "left_arm_tracker");

    // Define your node handles: YOU NEED AT LEAST ONE !
    ros::NodeHandle nh_glob, nh_loc("~") ;

    // Declare you publishers and service servers
    ros::ServiceClient client = nh_glob.serviceClient<dual_planar_arm_msgs::DualArmIK>("/right_arm_IK_service");

    // messages Initialization
    dual_planar_arm_msgs::DualArmIK service_msg;
    std::size_t cnt = 0;

    ros::Rate rate(100);   // Or other rate.
    while (ros::ok()) {
        ros::spinOnce();

        // Everything is done in the callback function. 
        // This loop only defines the frequency at which incoming messages are attended to.
        if (client.exists() && client.isValid()) {
            service_msg.request.goal.header.frame_id = "r_arm_base";
            bool successful = client.call(service_msg);
            if (!successful) {
                ROS_INFO("Request call was unsuccessfull...");
            } else {
                if (cnt % 10 == 0) {
                    // std::cout << service_msg.request.goal << std::endl;
                    std::size_t n_solutions = service_msg.response.solutions.size();
                    if (n_solutions > 0) {
                        std::cout << n_solutions << " found solutions. Joint Solution (1st) == " << service_msg.response.solutions[0];
                    } else {
                        std::cout << "No joint solutions..." << std::endl;
                    }
                }
            }
        } else {
            ROS_INFO("Service Client not ready or doesn't exist..");
        }

        cnt++;
        rate.sleep();
    }
}
