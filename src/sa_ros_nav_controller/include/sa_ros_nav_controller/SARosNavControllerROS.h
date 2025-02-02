#pragma once

#include <ros/ros.h>

#include <geometry_msgs/Twist.h> // Twist messages (linear & angular velocity)
#include <nav_msgs/Path.h> // Path message

#include <tf/transform_listener.h> // Odometry with tf

#include "sa_ros_nav_controller/GoToXYCommand.h" // GoToXY Command service
#include "sa_ros_nav_controller/FollowLineCommand.h" // FollowLine Command service
#include "sa_ros_nav_controller/FollowCircleCommand.h" // FollowCircle Command service
#include <std_srvs/Empty.h> // Stop Controller Command service

#include "sa_ros_nav_controller/SARosNavController.h"
#include "sa_ros_nav_controller/utils.h"

namespace sa_ros_nav_controller {

class SARosNavControllerROS {

private:
    ros::NodeHandle nh_; //Node handle

    ros::Publisher cmd_vel_pub_; //Publisher for the cmd_vel topic
    ros::Publisher ref_path_pub_; //Publisher for the ref_path topic
    ros::Subscriber pose_sub_; //Subscriver for the pose topic
    
    ros::ServiceServer gotoxy_srv_; //Service for the GotoXY Path Command
    ros::ServiceServer followline_srv_; //Service for the FollowLine Path Command
    ros::ServiceServer followcircle_srv_; //Service for the FollowCircle Path Command
    ros::ServiceServer stop_controller_srv_; //Service to stop the controller

    tf::TransformListener tf_listen_;
    std::string map_frame_id_;
    std::string base_frame_id_;

    SARosNavController nav_controller_;

    geometry_msgs::Twist velCommand_; //Linear and angular velocity in m/s

    bool stop_controller;

public:
    //ROS node initialization
    SARosNavControllerROS();
    ~SARosNavControllerROS() = default;

private:
    //Read parameters from config/*.yaml
    void readParam();

    //Function to update the robot pose using tf
    void updateRobotPose();

    //Callback function for GoToXY service
    bool srvGoToXYCommand(GoToXYCommand::Request& request, GoToXYCommand::Response& response);
    
    //Callback function for FollowLine service
    bool srvFollowLineCommand(FollowLineCommand::Request& request, FollowLineCommand::Response& response);
    
    //Callback function for FollowCircle service
    bool srvFollowCircleCommand(FollowCircleCommand::Request& request, FollowCircleCommand::Response& response);

    //Callback function for Stop service
    bool srvStopControllerCommand(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

    //Loop forever while sending control commands
    void controlLoop();

};

}
