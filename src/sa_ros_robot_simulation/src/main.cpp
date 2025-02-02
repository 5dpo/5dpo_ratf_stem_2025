#include "sa_ros_robot_simulation/SARosRobotSimulationROS.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "sa_ros_robot_simulation");

    sa_ros_robot_simulation::SARosRobotSimulationROS ros_robot_simulation;
    ros::spin();

    return 0;
}