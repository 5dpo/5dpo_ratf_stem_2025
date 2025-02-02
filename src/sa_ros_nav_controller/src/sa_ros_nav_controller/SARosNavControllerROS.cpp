#include <iostream>
#include <cmath>
#include <exception>

#include "sa_ros_nav_controller/SARosNavControllerROS.h"

namespace sa_ros_nav_controller {

SARosNavControllerROS::SARosNavControllerROS() {
    try {
        readParam();
    } catch (std::exception& e) {
        ROS_FATAL("[sa_ros_nav_controller] Error reading the node parameters (%s)", e.what());
        ros::shutdown();
    }

    nav_controller_ = SARosNavController();

    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    ref_path_pub_ = nh_.advertise<nav_msgs::Path>("ref_path", 1);

    gotoxy_srv_ = nh_.advertiseService("gotoxy_srv", &SARosNavControllerROS::srvGoToXYCommand, this);
    followline_srv_ = nh_.advertiseService("followline_srv", &SARosNavControllerROS::srvFollowLineCommand, this);
    followcircle_srv_ = nh_.advertiseService("followcircle_srv", &SARosNavControllerROS::srvFollowCircleCommand, this);

    stop_controller = true;
    stop_controller_srv_ = nh_.advertiseService("stop_controller_srv", &SARosNavControllerROS::srvStopControllerCommand, this);

    controlLoop();
}

void SARosNavControllerROS::readParam() {
    ros::NodeHandle nh_private("~");

    nh_private.param<std::string>("map_frame_id", map_frame_id_, "map");
    nh_private.param<std::string>("base_frame_id", base_frame_id_, "base_footprint");
}

void SARosNavControllerROS::updateRobotPose() {
    tf::StampedTransform tf_base2map;
    try {
        tf_listen_.lookupTransform(map_frame_id_, base_frame_id_, ros::Time(0), tf_base2map);
    } catch (tf::TransformException& e) {
        throw std::runtime_error(e.what());
    }

    nav_controller_.x_odo =  tf_base2map.getOrigin().getX();
    nav_controller_.y_odo =  tf_base2map.getOrigin().getY();
    nav_controller_.theta_odo = normAngRad(tf::getYaw(tf_base2map.getRotation()));
}

bool SARosNavControllerROS::srvGoToXYCommand(GoToXYCommand::Request& request, GoToXYCommand::Response& response) {
    stop_controller = false;
    
    nav_controller_.mode = 1;
    nav_controller_.xf = request.xf;
    nav_controller_.yf = request.yf;
    nav_controller_.thetaf = (request.thetaf * M_PIf32 / 180.0f);
    
    nav_msgs::Path ref_path;
    ref_path.header.frame_id = map_frame_id_;
    ref_path.header.seq = 0;
    ref_path.header.stamp = ros::Time::now();

    double delta_x = (request.xf - nav_controller_.x_odo)/19;
    double delta_y = (request.yf - nav_controller_.y_odo)/19;

    ref_path.poses.resize(20);
    ref_path.poses[0].pose.position.x = nav_controller_.x_odo;
    ref_path.poses[0].pose.position.y = nav_controller_.y_odo;
    for (int i=1; i<20; i++) {
            ref_path.poses[i].pose.position.x = ref_path.poses[i-1].pose.position.x + delta_x;
            ref_path.poses[i].pose.position.y = ref_path.poses[i-1].pose.position.y + delta_y;
    }
    ref_path_pub_.publish(ref_path);
        
    return true; 
}

bool SARosNavControllerROS::srvFollowLineCommand(FollowLineCommand::Request& request, FollowLineCommand::Response& response) {
    stop_controller = false;
    
    nav_controller_.mode = 2;
    nav_controller_.xi = request.xi;
    nav_controller_.yi = request.yi;
    nav_controller_.xf = request.xf;
    nav_controller_.yf = request.yf;
    nav_controller_.thetaf = (request.thetaf * M_PIf32 / 180.0f);
    
    nav_msgs::Path ref_path;
    ref_path.header.frame_id = map_frame_id_;
    ref_path.header.seq = 0;
    ref_path.header.stamp = ros::Time::now();

    double delta_x = (request.xf - request.xi)/19;
    double delta_y = (request.yf - request.yi)/19;

    ref_path.poses.resize(20);
    ref_path.poses[0].pose.position.x = request.xi;
    ref_path.poses[0].pose.position.y = request.yi;
    for (int i=1; i<20; i++) {
        ref_path.poses[i].pose.position.x = ref_path.poses[i-1].pose.position.x + delta_x;
        ref_path.poses[i].pose.position.y = ref_path.poses[i-1].pose.position.y + delta_y;
    }
    ref_path_pub_.publish(ref_path);
        
    return true;
}

bool SARosNavControllerROS::srvFollowCircleCommand(FollowCircleCommand::Request& request, FollowCircleCommand::Response& response) {
    stop_controller = false;

    nav_controller_.mode = 3;
    nav_controller_.xc = request.xc;
    nav_controller_.yc = request.yc;
    nav_controller_.r = request.r;
    nav_controller_.anglef = (request.anglef * M_PIf32 / 180.0f);
    nav_controller_.thetaf = (request.thetaf * M_PIf32 / 180.0f);
    
    nav_msgs::Path ref_path;
    ref_path.header.frame_id = map_frame_id_;
    ref_path.header.seq = 0;
    ref_path.header.stamp = ros::Time::now();

    double alfa = atan2(nav_controller_.y_odo - request.yc, nav_controller_.x_odo - request.xc);
    double beta = normAngRad((request.anglef * M_PIf32 / 180.0f));
    if(beta < alfa)
        beta = beta + 2*M_PI;
    double delta_angle = (beta - alfa)/100;
    double angle_itr = alfa;

    ref_path.poses.resize(100);
    for (int i=0; i<100; i++) {
        ref_path.poses[i].pose.position.x = request.xc + request.r * cos(angle_itr);
        ref_path.poses[i].pose.position.y = request.yc + request.r * sin(angle_itr);
        angle_itr = angle_itr + delta_angle;
    }
    ref_path_pub_.publish(ref_path);

    return true;
}

bool SARosNavControllerROS::srvStopControllerCommand(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
    stop_controller = true;

    return stop_controller;
}

void SARosNavControllerROS::controlLoop() {

    tf_listen_.waitForTransform(map_frame_id_, base_frame_id_, ros::Time::now(),
                    ros::Duration(5.0));

    ros::Rate loop_rate(30);
    while (ros::ok()) {
        ros::spinOnce();

        updateRobotPose();
        if(!stop_controller) {
            nav_controller_.getUpdatedVelocityVectors(velCommand_.linear.x, velCommand_.linear.y, velCommand_.angular.z);
            cmd_vel_pub_.publish(velCommand_);  
        }

        loop_rate.sleep();
    }
}

}

