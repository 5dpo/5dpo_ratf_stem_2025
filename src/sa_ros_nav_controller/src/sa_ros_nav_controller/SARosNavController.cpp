#include <iostream>
#include <cmath>

#include "sa_ros_nav_controller/SARosNavController.h"
#include "sa_ros_nav_controller/utils.h"

namespace sa_ros_nav_controller {

/*************************   TEMP   *********************************/
double vel_lin_nom = 0.2;
double vel_ang_nom = 0.2;
double k_error_lin = 5.0;
double k_vel_da = 3.0;
double tol_fin_dist = 0.02;
double tol_fin_theta = 1.0 * M_PIf32 / 180.0f;
double dist_da = 0.15;
double theta_da = 5.0 * M_PIf32 / 180.0f;
double dist_new_pose = 0.2;
double theta_new_pose = 7.5 * M_PIf32 / 180.0f;

//State Machine states
const int Go_Forward = 1;
const int De_Accel_Lin = 2;
const int Stop_Lin = 3;

const int Rotation = 1;
const int De_Accel_Rot = 2;
const int Stop_Rot = 3;

//Used to find fastest rotation
const int RotateRight = 1;
const int RotateLeft = -1;

/********************************************************************/

SARosNavController::SARosNavController() {
    x_odo = y_odo = theta_odo = 0.0;
    xi = yi = 0.0;
    xf = yf = thetaf = 0.0;
    xc = yc = anglef = 0.0;
    mode = 0;

    state_Lin = 0;
    state_Rot = 0;
}

void SARosNavController::getUpdatedVelocityVectors(double& v_x, double& v_y, double& v_theta) {
    //Debug
    double dist = sqrt(pow(xf - x_odo, 2) + pow(yf - y_odo, 2));
    double vel = sqrt(pow(v_x, 2) + pow(v_y, 2));
    std::cout << "Current (x, y, theta): (" << x_odo << ", " << y_odo << ", " << theta_odo << ")" << std::endl
              << "Desired (x, y, theta): (" << xf << ", " << yf << ", " << thetaf << ")" << std::endl
              << "Distance to Goal: " << dist << " m" << std::endl
              << "Velocity (V, Vn, W): (" << v_x << ", " << v_y << ", " << v_theta << ")" << std::endl
              << "Linear Velocity: " << vel << " m" << std::endl
              << std::endl;

    switch(mode) {
        case 1:
            gotoXY(v_x, v_y, v_theta);
            break;
        case 2:
            followLine(v_x, v_y, v_theta);
            break;
        case 3:
            followCircle(v_x, v_y, v_theta);
            break;
        default:
            v_x = v_y = v_theta = 0.0;
            break;
    } 

}

void SARosNavController::gotoXY(double& v_x, double& v_y, double& v_theta) {
    //Calculate errors
    double ang_target = atan2(yf - y_odo, xf - x_odo);
    double error_dist = dist(xf - x_odo, yf - y_odo);

    double error_final_rot = normAngRad(thetaf - theta_odo);

    //Find fastest rotation
    int rotate_to_final = 0;
    if (error_final_rot > 0.0)
        rotate_to_final = RotateRight;
    else
        rotate_to_final = RotateLeft;

    switch(state_Lin) {
        case Go_Forward:
            if (error_dist < tol_fin_dist)
                state_Lin = Stop_Lin;
            else if (error_dist < dist_da)
                state_Lin = De_Accel_Lin;
            break;
        case De_Accel_Lin:
            if (error_dist < tol_fin_dist)
                state_Lin = Stop_Lin;
            else if (error_dist > dist_new_pose)
                state_Lin = Go_Forward;
            break;
        case Stop_Lin:
            if (error_dist > dist_new_pose)
                state_Lin = Go_Forward;
            break;
        default:
            state_Lin = Stop_Lin;
    }

    //Transitions Rotation
    switch(state_Rot) {
        case Rotation:
            if (abs(error_final_rot) < tol_fin_theta)
                state_Rot = Stop_Rot;
            else if (abs(error_final_rot) < theta_da)
                state_Rot = De_Accel_Rot;
            break;
        case De_Accel_Rot:
            if (abs(error_final_rot) < tol_fin_theta)
                state_Rot = Stop_Rot;
            else if (abs(error_final_rot) > theta_new_pose)
                state_Rot = Rotation;
            break;
        case Stop_Rot:
            if (abs(error_final_rot) > theta_new_pose)
                state_Rot = Rotation;
            break;
        default:
            state_Rot = Stop_Rot;
    }

    //Outputs Linear Velocity
    switch(state_Lin) {
        case Go_Forward:
            v_x = vel_lin_nom * cos(ang_target - theta_odo);
            v_y = vel_lin_nom * sin(ang_target - theta_odo);
            break;
        case De_Accel_Lin:
            v_x = (vel_lin_nom / k_vel_da) * cos(ang_target - theta_odo);
            v_y = (vel_lin_nom / k_vel_da) * sin(ang_target - theta_odo);
            break;
        case Stop_Lin:
            v_x = 0.0;
            v_y = 0.0;
            break;
        default:
            v_x = 0.0;
            v_y = 0.0;
    }

    //Outputs Rotation
    switch(state_Rot) {
        case Rotation:
            v_theta = vel_ang_nom * rotate_to_final;
            break;
        case De_Accel_Rot:
            v_theta = (vel_ang_nom/k_vel_da) * rotate_to_final;
            break;
        case Stop_Lin:
            v_theta = 0.0;
            break;
        default:
            v_theta = 0.0;
    }
}

void SARosNavController::dist2Line(double& xr, double& yr) {
    double ux = (xf - xi) / dist(xf - xi, yf - yi);
    double uy = (yf - yi) / dist(xf - xi, yf - yi);

    double k = (x_odo*uy - y_odo*ux - xi*uy + yi*ux)/(pow(ux,2) + pow(uy,2));

    xr = x_odo - k*uy;
    yr = y_odo + k*ux;
}

void SARosNavController::followLine(double& v_x, double& v_y, double& v_theta) {
    //Calculate errors
    double error_dist = dist(xf - x_odo, yf - y_odo);
    double xr, yr;
    dist2Line(xr, yr);
    double alfa = atan2(yf - y_odo, xf - x_odo);

    double error_final_rot = normAngRad(thetaf - theta_odo);

    //Find fastest rotation
    int rotate_to_final = 0;
    if (error_final_rot > 0.0)
        rotate_to_final = RotateRight;
    else
        rotate_to_final = RotateLeft;

    switch(state_Lin) {
        case Go_Forward:
            if (error_dist < tol_fin_dist)
                state_Lin = Stop_Lin;
            else if (error_dist < dist_da)
                state_Lin = De_Accel_Lin;
            break;
        case De_Accel_Lin:
            if (error_dist < tol_fin_dist)
                state_Lin = Stop_Lin;
            else if (error_dist > dist_new_pose)
                state_Lin = Go_Forward;
            break;
        case Stop_Lin:
            if (error_dist > dist_new_pose)
                state_Lin = Go_Forward;
            break;
        default:
            state_Lin = Stop_Lin;
    }

    //Transitions Rotation
    switch(state_Rot) {
        case Rotation:
            if (abs(error_final_rot) < tol_fin_theta)
                state_Rot = Stop_Rot;
            else if (abs(error_final_rot) < theta_da)
                state_Rot = De_Accel_Rot;
            break;
        case De_Accel_Rot:
            if (abs(error_final_rot) < tol_fin_theta)
                state_Rot = Stop_Rot;
            else if (abs(error_final_rot) > theta_new_pose)
                state_Rot = Rotation;
            break;
        case Stop_Rot:
            if (abs(error_final_rot) > theta_new_pose)
                state_Rot = Rotation;
            break;
        default:
            state_Rot = Stop_Rot;
    }

    //Outputs Linear Velocity
    double VlinX, VlinY;
    VlinX = cos(alfa) + k_error_lin * (xr - x_odo);
    VlinY = sin(alfa) + k_error_lin * (yr - y_odo);
    VlinX = VlinX / sqrt(pow(VlinX,2) + pow(VlinY,2));
    VlinY = VlinY / sqrt(pow(VlinX,2) + pow(VlinY,2));
    switch(state_Lin) {
        case Go_Forward:
            v_x = vel_lin_nom * (VlinX * cos(theta_odo) + VlinY * sin(theta_odo));
            v_y = vel_lin_nom * (-VlinX * sin(theta_odo) + VlinY * cos(theta_odo));
            break;
        case De_Accel_Lin:
            v_x = (vel_lin_nom/k_vel_da) * (VlinX * cos(theta_odo) + VlinY * sin(theta_odo));
            v_y = (vel_lin_nom/k_vel_da) * (-VlinX * sin(theta_odo) + VlinY * cos(theta_odo));
            break;
        case Stop_Lin:
            v_x = 0.0;
            v_y = 0.0;
            break;
        default:
            v_x = 0.0;
            v_y = 0.0;
    }

    //Outputs Rotation
    switch(state_Rot) {
        case Rotation:
            v_theta = vel_ang_nom * rotate_to_final;
            break;
        case De_Accel_Rot:
            v_theta = (vel_ang_nom/k_vel_da) * rotate_to_final;
            break;
        case Stop_Lin:
            v_theta = 0.0;
            break;
        default:
            v_theta = 0.0;
    }
}

void SARosNavController::dist2Arc(double& xr, double& yr) {
    double ux = (x_odo - xc) / dist(x_odo - xc, y_odo - yc);
    double uy = (y_odo - yc) / dist(x_odo - xc, y_odo - yc);

    xr = xc + r * ux;
    yr = yc + r * uy;
}

void SARosNavController::followCircle(double& v_x, double& v_y, double& v_theta) {
    //Calculate errors
    double xr, yr;
    dist2Arc(xr, yr);
    xf = xc + r * cos(anglef);
    yf = yc + r * sin(anglef);

    double error_final_rot = normAngRad(thetaf - theta_odo);

    //Find fastest rotation
    int rotate_to_final = 0;
    if (error_final_rot > 0.0)
        rotate_to_final = RotateRight;
    else
        rotate_to_final = RotateLeft;

    double alfa = atan2(y_odo - yc, x_odo - xc);
    double beta = atan2(yf - yc, xf - xc);
    if(beta < alfa)
        beta = beta + 2*M_PI;
    double error_dist = (beta - alfa) * r; // angular distance

    alfa = alfa + M_PI/2; // circle tangent

    switch(state_Lin) {
        case Go_Forward:
            if (error_dist < tol_fin_dist)
                state_Lin = Stop_Lin;
            else if (error_dist < dist_da)
                state_Lin = De_Accel_Lin;
            break;
        case De_Accel_Lin:
            if (error_dist < tol_fin_dist)
                state_Lin = Stop_Lin;
            else if (error_dist > dist_new_pose)
                state_Lin = Go_Forward;
            break;
        case Stop_Lin:
            if (error_dist > dist_new_pose)
                state_Lin = Go_Forward;
            break;
        default:
            state_Lin = Stop_Lin;
    }

    //Transitions Rotation
    switch(state_Rot) {
        case Rotation:
            if (abs(error_final_rot) < tol_fin_theta)
                state_Rot = Stop_Rot;
            else if (abs(error_final_rot) < theta_da)
                state_Rot = De_Accel_Rot;
            break;
        case De_Accel_Rot:
            if (abs(error_final_rot) < tol_fin_theta)
                state_Rot = Stop_Rot;
            else if (abs(error_final_rot) > theta_new_pose)
                state_Rot = Rotation;
            break;
        case Stop_Rot:
            if (abs(error_final_rot) > theta_new_pose)
                state_Rot = Rotation;
            break;
        default:
            state_Rot = Stop_Rot;
    }

    //Outputs Linear Velocity
    double VlinX, VlinY;
    VlinX = cos(alfa) + k_error_lin * (xr - x_odo);
    VlinY = sin(alfa) + k_error_lin * (yr - y_odo);
    VlinX = VlinX / sqrt(pow(VlinX,2) + pow(VlinY,2));
    VlinY = VlinY / sqrt(pow(VlinX,2) + pow(VlinY,2));
    switch(state_Lin) {
        case Go_Forward:
            v_x = vel_lin_nom * (VlinX * cos(theta_odo) + VlinY * sin(theta_odo));
            v_y = vel_lin_nom * (-VlinX * sin(theta_odo) + VlinY * cos(theta_odo));
            break;
        case De_Accel_Lin:
            v_x = (vel_lin_nom/k_vel_da) * (VlinX * cos(theta_odo) + VlinY * sin(theta_odo));
            v_y = (vel_lin_nom/k_vel_da) * (-VlinX * sin(theta_odo) + VlinY * cos(theta_odo));
            break;
        case Stop_Lin:
            v_x = 0.0;
            v_y = 0.0;
            break;
        default:
            v_x = 0.0;
            v_y = 0.0;
    }

    //Outputs Rotation
    switch(state_Rot) {
        case Rotation:
            v_theta = vel_ang_nom * rotate_to_final;
            break;
        case De_Accel_Rot:
            v_theta = (vel_ang_nom/k_vel_da) * rotate_to_final;
            break;
        case Stop_Lin:
            v_theta = 0.0;
            break;
        default:
            v_theta = 0.0;
    }
}


}

