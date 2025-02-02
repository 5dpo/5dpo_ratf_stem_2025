#include <iostream>
#include <cmath>
#include <exception>

#include "sa_ros_robot_simulation/SARosRobotSimulationROS.h"

namespace sa_ros_robot_simulation {

SARosRobotSimulationROS::SARosRobotSimulationROS() {
    try {
        readParam();
    } catch (std::exception& e) {
        ROS_FATAL("[sa_ros_robot_simulation] Error reading the node parameters (%s)", e.what());
        ros::shutdown();
    }

    position_pub_ = nh_.advertise<geometry_msgs::PointStamped>("robot_position", 1);

    cmd_vel_sub_ = nh_.subscribe("cmd_vel", 1, &SARosRobotSimulationROS::subCmdVel, this);

    t_last = ros::Time::now().toSec();
    x_odo = y_odo = theta_odo = 0.0;
    vx_last = vy_last = vtheta_last = 0.0;

    pubPoseLoop();
}

void SARosRobotSimulationROS::readParam() {
  ros::NodeHandle nh_private("~");

  nh_private.param<std::string>("map_frame_id", map_frame_id_, "map");
  nh_private.param<std::string>("base_frame_id", base_frame_id_, "base_footprint");
}

void SARosRobotSimulationROS::subCmdVel(const geometry_msgs::Twist& msg) {
    double t = ros::Time::now().toSec();
    double dt = t - t_last;
    
    x_odo += vx_last * dt;
    y_odo += vy_last * dt;
    theta_odo += vtheta_last * dt;

    t_last = t;
    vx_last = msg.linear.x * cos(theta_odo) - msg.linear.y * sin(theta_odo);
    vy_last = msg.linear.x * sin(theta_odo) + msg.linear.y * cos(theta_odo);
    vtheta_last = msg.angular.z;
}

void SARosRobotSimulationROS::pubPoseLoop() {

    ros::Rate loop_rate(50);
    while (ros::ok()) {
        ros::spinOnce();
      
        tf::StampedTransform map2base_tf;
        map2base_tf.setOrigin(tf::Vector3(x_odo, y_odo, 0.0));
        map2base_tf.setRotation(tf::createQuaternionFromYaw(theta_odo));
        map2base_tf.stamp_ = ros::Time::now();
        map2base_tf.frame_id_ = map_frame_id_;
        map2base_tf.child_frame_id_ = base_frame_id_;
        tf_broad_.sendTransform(map2base_tf);

        geometry_msgs::PointStamped robot_pose;
        robot_pose.header.frame_id = map_frame_id_;
        robot_pose.header.stamp = map2base_tf.stamp_;
        robot_pose.point.x = x_odo;
        robot_pose.point.y = y_odo;

        position_pub_.publish(robot_pose);

        loop_rate.sleep();
    }
}

}
