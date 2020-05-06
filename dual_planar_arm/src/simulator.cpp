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

// You may have a number of globals here.
//...

// Callback functions...

bool jointCommandReceived = false ;
sensor_msgs::JointState jointCommand ;
void jointCommandCallback( sensor_msgs::JointState jointCommandMsg ){
    jointCommandReceived = true            ; 
    jointCommand         = jointCommandMsg ;
}



int main (int argc, char** argv)
{

	//ROS Initialization
    ros::init(argc, argv, "simulator");

    // Define your node handles: YOU NEED AT LEAST ONE !
    ros::NodeHandle nh_glob, nh_loc("~") ;

    // Read the node parameters if any
    // ...

    // Declare your node's subscriptions and service clients
    ros::Subscriber subJointCommands = nh_glob.subscribe<sensor_msgs::JointState>("/joint_command",1,jointCommandCallback) ;

    // Declare you publishers and service servers
    ros::Publisher pubJointState = nh_glob.advertise<sensor_msgs::JointState>("/joint_states",1) ;

    // Intial state
    sensor_msgs::JointState robotState ;
    robotState.header.stamp = ros::Time::now() ;
    robotState.name.push_back("r_shoulder") ;
    robotState.name.push_back("r_elbow") ;
    robotState.name.push_back("l_shoulder") ;
    robotState.name.push_back("l_elbow") ;
    robotState.position.push_back(0.0) ;
    robotState.position.push_back(0.0) ;
    robotState.position.push_back(0.0) ;
    robotState.position.push_back(0.0) ;

    ros::Rate rate(100);   
    while (ros::ok()){
        ros::spinOnce();

        // Now change those states that are modified by the joint command.
        if( jointCommandReceived ){
            jointCommandReceived = false ;
            for( int i=0 ; i<jointCommand.name.size() ; i++ ){
                int j ;
                for( j=0 ; (j<robotState.name.size()) && (robotState.name[j]!=jointCommand.name[i]) ; j++ ) ;
                if( j<robotState.name.size() ){
                    robotState.position[j] = jointCommand.position[i] ;
                }else{
                    ROS_INFO("Command name not found.");
                }
            }
        }

        // Publish the new joint command 
        robotState.header.stamp = ros::Time::now() ;
        pubJointState.publish(robotState) ;

        rate.sleep();
    }
}

