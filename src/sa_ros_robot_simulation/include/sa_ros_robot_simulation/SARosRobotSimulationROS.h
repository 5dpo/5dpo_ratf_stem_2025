#pragma once

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>

#include <tf/transform_broadcaster.h>

namespace sa_ros_robot_simulation {

class SARosRobotSimulationROS {

private:
    ros::NodeHandle nh_;

    ros::Subscriber cmd_vel_sub_;
    ros::Publisher position_pub_;

    tf::TransformBroadcaster tf_broad_;

    std::string map_frame_id_;
    std::string base_frame_id_;

    double x_odo, y_odo, theta_odo;
    double t_last;
    double vx_last, vy_last, vtheta_last;

public:
    //ROS node initialization
    SARosRobotSimulationROS();
    ~SARosRobotSimulationROS() = default;

private:
    //Read parameters from config/*.yaml
    void readParam();

    void subCmdVel(const geometry_msgs::Twist& msg);

    void pubPoseLoop();
};

}