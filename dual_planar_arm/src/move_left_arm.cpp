/**
 * \file
 * \brief 
 * \author Gaetan GARCIA
 * \version 0.1
 * \date 26 April 2020
 * 
 * \param[in] 
 * 
 * Subscribes to: <BR>
 *    "/joint_states" : robot joint state
 *    "/joint_commands" : position commands sent.
 * 
 * Publishes to: <BR>
 *    "/joint_states": robot joint state
 *
 * Description
 *    Sets the robot position to the values sent in the joint commands, while leaving the 
 *  joints not concerned untouched. Robot is set to the position instantly, no control.
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

// Include here the ".h" files corresponding to the topic type you use.
#include <sensor_msgs/JointState.h>

int main (int argc, char** argv)
{

	//ROS Initialization
    ros::init(argc, argv, "move_left_arm");

    // Define your node handles: YOU NEED AT LEAST ONE !
    ros::NodeHandle nh_glob, nh_loc("~") ;

    // Declare you publishers and service servers
    ros::Publisher pubJointCommand = nh_glob.advertise<sensor_msgs::JointState>("/joint_command",1) ;

    // Intial state
    sensor_msgs::JointState jointCommand ;
    jointCommand.header.stamp = ros::Time::now() ;
    jointCommand.name.push_back("l_shoulder") ;
    jointCommand.name.push_back("l_elbow") ;
    jointCommand.position.push_back(0.0) ;
    jointCommand.position.push_back(0.0) ;

    double shoulderPos = 0.0, elbowPos = 0.0 ;

    double shoulderAmplitude = M_PI/6 ;
    double elbowAmplitude    = M_PI/2 ;
    double shoulderSpeed     = M_PI/8 ;  // rad/s
    double elbowSpeed        = shoulderSpeed * elbowAmplitude / shoulderAmplitude ;
    
    ros::Time prevTime ;
    prevTime = ros::Time::now() ;

    ros::Rate rate(50);   
    while (ros::ok()){
        ros::spinOnce();

        ros::Time currentTime = ros::Time::now() ;
        ros::Duration timeElapsed = currentTime - prevTime ;
        shoulderPos += timeElapsed.toSec()*shoulderSpeed ;
        elbowPos    += timeElapsed.toSec()*elbowSpeed    ;
        prevTime = currentTime ;
        if( abs(shoulderPos)>shoulderAmplitude || abs(elbowPos)>elbowAmplitude ){
            shoulderPos = shoulderPos/abs(shoulderPos) * shoulderAmplitude ;
            elbowPos    = elbowPos/abs(elbowPos)       * elbowAmplitude    ;
            shoulderSpeed *= (-1) ;
            elbowSpeed    *= (-1) ;
        }
        jointCommand.position[0] = shoulderPos ;
        jointCommand.position[1] = elbowPos    ;
 
        // Publish the new joint command 
        jointCommand.header.stamp = ros::Time::now() ;
        pubJointCommand.publish(jointCommand) ;

        rate.sleep();
    }
}

