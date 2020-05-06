#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_listener");

  ros::NodeHandle nh_glob, nh_loc("~") ;

  /*
  ros::service::waitForService("spawn");
  ros::ServiceClient add_turtle = 
    node.serviceClient<turtlesim::Spawn>("spawn");
  turtlesim::Spawn srv;
  add_turtle.call(srv);
  */
  // Declare you publishers and service servers
  ros::Publisher pubGoalPose = nh_glob.advertise<geometry_msgs::PoseStamped>("/goal_pos",1) ;
  tf::TransformListener listener;

  listener.waitForTransform("/r_arm_base", "/r_arm_goal", 
                                ros::Time(0), ros::Duration(5.0), ros::Duration(2));

  ros::Rate rate(10.0);
  while (ros::ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/r_arm_base", "/r_arm_goal",
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
    
    pubGoalPose.publish(goalPose);

    rate.sleep();
  }
  return 0;
};