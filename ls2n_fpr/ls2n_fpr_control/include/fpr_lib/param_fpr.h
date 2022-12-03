/** @file param_fpr.h
 *  @brief Structure for FPR parameters 
 *
 *  @author Shiyu-LIU
 *  @version: v2.0
 *  @date: 11 March 2021
 */
#pragma once

#include <ostream>
#include "common_fpr.h"

namespace FPR{

struct ParamFPR
{
// Structure for constant parameters of FPR: dynamic parameters and control gains
    // Mass constants:
    double mp; // mass of the platform
    double mleg; // mass of the leg
    double m_leg[NUM_LEG]; // overall mass of legs (leg mass + drone mass)
    double m_drones[NUM_LEG]; // mass of drones

    // Center of Mass, first moment of inertia(= mass* COM position)
    Eigen::Vector3d sp; // center of mass position of the platform
    Eigen::Vector3d sleg; // center of mass position of the leg

    Eigen::Vector3d ms_p; // first moment of platform
    Eigen::Vector3d ms_leg[NUM_LEG]; // overall COM position of legs w.r.t leg's frame (considering drone's and leg's masses) 

    // Moment of inertia:
    Eigen::Matrix3d Ip; // moment of inertia of the platform     
    Eigen::Matrix3d Ileg; // moment of inertia of each leg (equal for different legs)
    Eigen::Matrix3d I_leg[NUM_LEG]; // overall moments of inertia of legs w.r.t each leg's frame

    // Geometric constants:
    double l_leg; // leg's length 
    double r_p; // platform's radius (distance from its center to that of the revolute joint)
    
    double alpha[NUM_LEG]; // constant angles representing angle between x axis of platform and each leg's direction
    double beta[NUM_LEG]; // constant angles between MAVROS x axis and leg's direction expressed in MAVROS z axis

    Eigen::Vector3d pos_ri[NUM_LEG]; // revolute joint positions expressed in platform frame (which are constants)

    // Gravity
    double gravity;

    struct ControlGain{
        /* PID gains */
        // position_platform:
        double kp_p[3]; // proportional gains respectively on x,y,z axis
        double kd_p[3]; // derivative gains
        double ki_p[3]; // integral gains
        // orientation_platform:
        double kp_o[3];
        double kd_o[3];  
        double ki_o[3];
        // angle_leg:
        double kp_l;
        double kd_l;
        double ki_l;

        /* Impedance gains */
        double mv_p;
        double dv_p;
        double kv_p;
        double mv_o;
        double dv_o;
        double kv_o;
        double mv_l;
        double dv_l;
        double kv_l;
    } ctrl_gain;

    struct ObserverGain{
        double k_p;
        double k_o;
        double k_l;
    } observer_gain;

    ParamFPR()
    {
        for (int i=0; i < NUM_LEG; i++)
            alpha[i] = i*(2.0/NUM_LEG)*M_PI; // in case NUM_LEG=3, alpha = {0, 2/3*pi, 4/3*pi}

        sp.setZero(); sleg.setZero();
        Ip.setZero(); Ileg.setZero();

        gravity = 9.81; // set the gravity value by default
    };

    void computeParam();
    void setParam(ParamFPR &param) { *this = param; };
    void printParam(std::ostream *pOut);
};

}