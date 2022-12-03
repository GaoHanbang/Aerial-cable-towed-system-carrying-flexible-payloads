/** @file modelfpr.h
 *  @brief Class for kinematic and dynamic modeling of FPR 
 *
 *  @author Shiyu-LIU
 *  @version: v2.0
 *  @date: 11 March 2021
 */
#pragma once

#include <eigen3/Eigen/Dense>
#include "common_fpr.h"
#include "param_fpr.h"

namespace FPR{

class ModelFPR
// Class for kinematic and dynamic model of FPR (passive architecture), 
// including a Jacobian matrix, an inertia matrix and a vector of Coriolis, centrifugal and gravitational effects.
{
public:

typedef Eigen::Matrix<double, DIM_ACT, DIM_DOF> JacobianType;

    JacobianType JacobianMat; // jacobian matrix
    JacobianType invTransJacobianMat; // inverse of transpose of jacobian matrix

    DofMatType InertiaMat; // inertia matrix
    DofVecType CoriolisVec; // coriolis vector
    DofMatType CoriolisMat; // coriolis matrix (CoriolisVec = CoriolisMat*q_dot + GravityEffects) 
    DofVecType GravityVec; // only used with coriolis matrix (coriolis vector has included this if gravity is not zero)

    DofMatType InertiaMat_PassArchi; // inertial matrix for passive architecture (without drones)
    DofVecType CoriolisVec_PassArchi; // coriolis vector for passive architecture

    Eigen::Matrix3d Rp; // rotation matrix from F0 to Fp
    Eigen::Matrix3d Rleg[NUM_LEG]; // rotation matrix from Fp to Fli

    ParamFPR param;

    ModelFPR(ParamFPR& _pm) : param(_pm)
    {
        JacobianMat.setZero();
        InertiaMat.setZero();
        CoriolisVec.setZero();
        Rp.setZero();
    }

    void resetParam(ParamFPR _pm)
    {
        param = _pm;
        param.computeParam();
    }

    // Update kinematic model (computation of Jacobian matrix)
    void updateKinematics(const StateVecType& current_pos);
    // Update Dynamic model with fast method (computation of Inertia matrix and Coriolis vector, in form of M*q_dd + c)
    void updateDynamics(const StateVecType& current_pos, const DofVecType& current_vel);
    // Update Dynamic model with Coriolis separation (computation of Inertia matrix, Coriolis matrix and Gravity term, in form of M*qdd + C*qd + G)
    void updateDynamicsCoriolis(const StateVecType& current_pos, const DofVecType& current_vel);
    // Compute the inverse of the transpose of Jacobian
    JacobianType invTransJacobian();

private:
    void computeKinematicModel(const StateVecType& current_pos);
    void computeDynamicModelOld(const StateVecType& current_pos, const DofVecType& current_vel);
    void computeDynamicModelFast(const StateVecType& current_pos, const DofVecType& current_vel);
    void computeDynamicModelFull(const StateVecType& current_pos, const DofVecType& current_vel);
    void computeGravityTerm(const StateVecType& current_pos);
};

}