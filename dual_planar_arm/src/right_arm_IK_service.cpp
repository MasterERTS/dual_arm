/**
 * \file right_arm_IK_service
 * \brief IK service of the right arm of the dual planar arm robot.
 * \author G. Garcia
 * \version 0.1
 * \date 27 April 2020
 * 
 * \param[in] 
 *      None
 * Subscribes to: <BR>
 *    ° /right_arm_IK_service, absolute topic name.
 * 
 * Publishes to: <BR>
 *    ° None
 *
 * Description
 *      Calculates the Inverse Geometric Model of the right arm of the dual planar arm robot.
 *  All robot characteristics (arm lengths and joint limits are hard codeded in the souce 
 *  file. This was done for lack of time but is not an example to follow. Those parameters 
 *  should be retrived from the URDF file using a URDF parser.
 */


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

// Include here the ".h" files corresponding to the topic type you use.
#include <dual_planar_arm_msgs/DualArmIK.h>

// You may have a number of globals here.
//...

// Callback functions...

#define D1 1.0
#define D2 1.0 
// Symmetrical joint limits: min = -max.
#define MAX1 (3*M_PI/4) 
#define MAX2 (3*M_PI/4)
// Reach is defined without taking into account joint limits.
#define MAX_REACH (D1+D2)
#define MIN_REACH abs(D1-D2)

bool right_arm_IK_service(
        dual_planar_arm_msgs::DualArmIK::Request &ik_request,
        dual_planar_arm_msgs::DualArmIK::Response &ik_response ){
        
    // Make some elementary checks on the request part
    if( ik_request.goal.header.frame_id.empty() ){
        ROS_ERROR("Empty frame id in IK service header.") ;
        return false ;
    }
    if( isnan(ik_request.goal.point.x) || isnan(ik_request.goal.point.y) ){
        // z is ignored anyway.
        ROS_ERROR("Nan in IK service request.") ;
        return false ;
    }
    ik_response.solutions.clear() ;
    double x = ik_request.goal.point.x , y = ik_request.goal.point.y ;
    if( sqrt(x*x+y*y) > MAX_REACH ){
        // Point is too far to be reached.
        ROS_INFO("Too far.");
        return true ;  // The solution vectors will be empty, but request was valid.
    }
    if( sqrt(x*x+y*y) < MIN_REACH ){
        // Point is too close to the arm basee.
        ROS_INFO("Too close.");
        return true ;  // The solution vectors will be empty, but request was valid.
    }
    // Otherwise calculate angles
    // Calculate cos_teta2 and both solutions for sin_teta2

    double ct2 = (x*x + y*y - D1*D1 - D2*D2) / (2*D1*D2) ;
    double st2_1 = sqrt(1 - ct2*ct2) ;
    double st2_2 = -st2_1            ;
    
    // The two angles for teta2

    double teta2_1 = atan2(st2_1, ct2) ;
    double teta2_2 = - teta2_1         ;
    
    // Determination of teta1 for the first solution of teta2. s stands for sine, c for cosine. st1 is then sin(teta1).

    double st1 =   (y * (D1 + D2*ct2) - x*D2*st2_1) / ((D1 + D2*ct2) * (D1 + D2*ct2) + D2*D2*st2_1*st2_1) ;
    double ct1 =   (x * (D1 + D2*ct2) + y*D2*st2_1) / ((D1 + D2*ct2) * (D1 + D2*ct2) + D2*D2*st2_1*st2_1) ;
    double teta1_1 = atan2(st1, ct1) ;

    // Determination of teta1 for the second solution of teta2

    st1 =   (y * (D1 + D2*ct2) - x*D2*st2_2) / ((D1 + D2*ct2) * (D1 + D2*ct2) + D2*D2*st2_2*st2_2) ;
    ct1 =   (x * (D1 + D2*ct2) + y*D2*st2_2) / ((D1 + D2*ct2) * (D1 + D2*ct2) + D2*D2*st2_2*st2_2) ;
    double teta1_2 = atan2(st1,ct1) ;
    
    //std::cout << "Angles 1: " << teta1_1*180/M_PI << " deg.,  " << teta2_1*180/M_PI << " deg.  " << std::endl ;
    //std::cout << "Angles 2: " << teta1_2*180/M_PI << " deg.,  " << teta2_2*180/M_PI << " deg.  " << std::endl ;
    //std::cout << "---" << std::endl ;
    
    // Compare solutions to joint limits and add to response if compatible.
    
    sensor_msgs::JointState js_message ;
    
    if( (teta1_1>=-MAX1) && (teta1_1<=MAX1) && (teta2_1>=-MAX2) && (teta2_1<=MAX2) ){
        // Solution 1 is valid. Push it into response.  
        js_message.header.stamp = ros::Time::now() ;
        js_message.name.push_back("r_shoulder") ;
        js_message.position.push_back(teta1_1) ;
        js_message.name.push_back("r_elbow") ;
        js_message.position.push_back(teta2_1) ;
        ik_response.solutions.push_back(js_message) ;
        js_message.name.clear() ;
        js_message.position.clear() ;
    }
    if( (teta1_2>=-MAX1) && (teta1_2<=MAX1) && (teta2_2>=-MAX2) && (teta2_2<=MAX2) ){
        // Solution 2 is valid. Push it into response.  
        // Don't time stamp, so the time stamps for both soutions agree.
        js_message.name.push_back("r_shoulder") ;
        js_message.position.push_back(teta1_2) ;
        js_message.name.push_back("r_elbow") ;
        js_message.position.push_back(teta2_2) ;
        ik_response.solutions.push_back(js_message) ;
        js_message.name.clear() ;
        js_message.position.clear() ;        

    }
}

int main (int argc, char** argv)
{

	//ROS Initialization
    ros::init(argc, argv, "right_arm_IK_service");

    // Define your node handles: YOU NEED AT LEAST ONE !
    ros::NodeHandle nh_glob, nh_loc("~") ;

    // Declare you publishers and service servers
    ros::ServiceServer service = nh_glob.advertiseService("/right_arm_IK_service", right_arm_IK_service);

    ros::Rate rate(100);   // Or other rate.
    while (ros::ok()){
        ros::spinOnce();

        // Everything is done in the callback function. 
        // This loop only defines the frequency at which incoming messages are attended to.

        rate.sleep();
    }
}

