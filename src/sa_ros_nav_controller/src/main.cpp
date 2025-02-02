#include "sa_ros_nav_controller/SARosNavControllerROS.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "sa_ros_nav_controller");

    sa_ros_nav_controller::SARosNavControllerROS ros_nav_controller;
    ros::spin();

    return 0;
}