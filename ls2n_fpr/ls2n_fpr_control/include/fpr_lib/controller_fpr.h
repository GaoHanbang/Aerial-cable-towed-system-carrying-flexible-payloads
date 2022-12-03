/** @file controllerFPR.h
 *  @brief Class for controlling the Flying Parallel Robot (FPR)
 *
 *  @author Shiyu-LIU
 *  @version: v2.0
 *  @date: 11 March 2021
 */
#pragma once

#include <eigen3/Eigen/Dense>

#include "common_fpr.h"
#include "state_fpr.h"
#include "model_fpr.h"

namespace FPR {

class ControllerFPR
{

public:
    enum ControllerType
    {
        PD_CTC,
        PID_CTC,
        IMPEDANCE
    };

    struct ControlErrorState
    {
        DofVecType pose_error;
        DofVecType accum_error; // accumulated pose error
        DofVecType velocity_error; // difference between desired and acutal velocity
    };

    struct DroneTarget
    {  
        double thrust;
        Eigen::Quaterniond attitude;
    };

    // Constructor
    ControllerFPR(ModelFPR& _mdl, ControllerType ctrlType);
    // Destructor
    ~ControllerFPR(){};
    // Spin the controller once
    bool spinController(std::string& msgerr);
    // Update the desired trajectory
    void updateTrajectory(const StateVecType& _des_pose, const DofVecType& _des_vel, const DofVecType& _des_acc);
    // Update the current robot pose
    void updateCurrentPose(const StateVecType& _pose);
    // Update the current velocity
    void updateCurrentVelocity(const DofVecType& _vel);
    // Get drone thrust/attitude commands
    void getDroneTargets(std::vector<DroneTarget>& _target);
    // Get the computed actuation wrench
    DofVecType getActuationWrench() { return actuation_wrench; };
    // Get the computed thrust force (thrust vectors of the drones)
    ActVecType getThrustForce() { return thrust_force; };
    // Get the computed interaction force between the platform and the drones
    ActVecType getInteractionForce() { return interaction_force; };
    // Get auxiliary control output
    DofVecType getAuxControlOutput() { return aux_control_output; };
    // Get the controller errors
    ControlErrorState getControlError() { return control_error; };
    // Update external wrench estimation, (full_estimate: total external wrench, partial_estimate: external wrench due to the interaction)
    void updateExternalWrench(const DofVecType& estimated);
    // Update desired external wrench
    void updateDesiredWrench(const DofVecType& desired);

protected:

    // Kinematic and dynamic model of FPR
    std::shared_ptr<ModelFPR> model;
    // Current FPR pose
    StateVecType pose; 
    // Desired FPR pose
    StateVecType des_pose; 
    // Current FPR velocity
    DofVecType velocity;
    // Desired FPR velocity
    DofVecType des_velocity;
    // Desired FPR acceleration
    DofVecType des_acceleration;

    // Actuation wrench that drives the FPR (including force and moment exerted on the platform and moments of legs)
    DofVecType actuation_wrench;
    // Thrust force vector
    ActVecType thrust_force;
    // Interaction force between the platform and the drones (computed by the desired commands)
    ActVecType interaction_force;

    // Drone desired targets (desired thrust and attitude)
    DroneTarget drone_target[NUM_LEG];

    // Controller type
    ControllerType controller_type;
    // Control error
    ControlErrorState control_error;
    // Auxiliary control output (PID or Impedance)
    DofVecType aux_control_output;

    // Control gain structure
    struct ControlGains
    {
        /* PID control gains */
        Eigen::Matrix<double, DIM_DOF, DIM_DOF> PID_Kp;
        Eigen::Matrix<double, DIM_DOF, DIM_DOF> PID_Kd;
        Eigen::Matrix<double, DIM_DOF, DIM_DOF> PID_Ki;
        /* Impedance control gains */
        Eigen::Matrix<double, DIM_DOF, DIM_DOF> Mv;
        Eigen::Matrix<double, DIM_DOF, DIM_DOF> Dv;
        Eigen::Matrix<double, DIM_DOF, DIM_DOF> Kv;
    } control_gains;

    // External wrench structure
    struct ExternalWrench
    {
        DofVecType desired; // desired wrench on the FPR
        DofVecType estimated; // estimated external wrench
    } external_wrench;

    bool traj_updated;
    bool pose_updated;
    bool vel_updated;

    // Compute the drone targets
    void computeDroneTarget();
    // Motion control law (PID + feedback linearization)
    void doMotionControl();
    // Impedance control law
    void doImpedanceControl();
    
    DofVecType computeControlError();
    DofVecType computePDControlLaw();
    DofVecType computePIDControlLaw();
};

}