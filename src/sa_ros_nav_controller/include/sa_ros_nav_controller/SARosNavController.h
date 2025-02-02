#pragma once

#include <geometry_msgs/Twist.h> // Twist messages (linear & angular velocity)
#include <geometry_msgs/Pose2D.h> // x, y position and theta orientation
#include <geometry_msgs/Point.h> // x, y position

namespace sa_ros_nav_controller {

class SARosNavController {

public:
    int mode; // Trajectory mode
    double x_odo, y_odo, theta_odo; // Current robot pose
    double xi, yi; // FollowLine inicial point
    double xf, yf, thetaf; // GoToXY, FollowLine and FollowCircle final pose
    double xc, yc, r, anglef; // FollowCircle circle parameters

    int state_Lin; //State Machine 1 (Linear motion)
    int state_Rot; //State Machine 2 (Rotating motion)


public:
    //Class initialization
    SARosNavController();
    ~SARosNavController() = default;

    //Update reference velocity vectors
    void getUpdatedVelocityVectors(double& v_x, double& v_y, double& v_theta);

private:
    //Calculate velocities to reach one pose (x, y, theta)
    void gotoXY(double& v_x, double& v_y, double& v_theta);

    //Calculate velocities to follow one line (xi, yi, xf, yf, thetaf)
    void followLine(double& v_x, double& v_y, double& v_theta);

    //Calculate velocities to follow one circle (xc, yc, r, anglef, thetaf)
    void followCircle(double& v_x, double& v_y, double& v_theta);

    //Calculate closest point to one Line
    void dist2Line(double& xr, double& yr);

    //Calculate closest point to one Circle
    void dist2Arc(double& xr, double& yr);

};

}
