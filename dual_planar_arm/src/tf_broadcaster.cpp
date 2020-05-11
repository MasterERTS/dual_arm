/**
 * \file
 * \brief 
 * \author 
 * \version 0.1
 * \date 
 * 
 * \param[in] 
 * 
 * Subscribes to: <BR>
 *    ° 
 * 
 * Publishes to: <BR>
 *    ° 
 *
 * Description
 *
 */


//Cpp
#include <sstream>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <math.h>

//ROS
#include "ros/ros.h"
#include <tf/transform_broadcaster.h>


std::string turtle_name;

int main (int argc, char** argv)
{
	//ROS Initialization
    ros::init(argc, argv, "tf_broadcaster");

    // Define your node handles: YOU NEED AT LEAST ONE !
    ros::NodeHandle nh_glo, nh_loc("~") ;

    tf::TransformBroadcaster br;
    tf::Transform transform;

    ros::Rate rate(50);   // Or other rate.
    while (ros::ok()){
        ros::spinOnce();
        transform.setOrigin( tf::Vector3(1.0, 0.0, 0.0) );
        tf::Quaternion q;
        q.setRPY(0, 0, 0);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "l_forearm", "/r_arm_goal"));

        rate.sleep();
    }
}

