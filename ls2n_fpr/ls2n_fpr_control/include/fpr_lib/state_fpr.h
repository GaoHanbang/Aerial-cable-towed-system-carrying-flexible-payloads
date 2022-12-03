/** @file state_fpr.h
 *  @brief Structure for defining FPR state, including the FPR pose and the drone state
 *
 *  @author Shiyu-LIU Damien-SIX
 *  @version: v2.1
 *  @date: 15 March 2021
 */
#pragma once
#include "common_fpr.h"
#include "param_fpr.h"
#include "model_fpr.h"

namespace FPR 
{

struct DroneState
{
    Eigen::Vector3d position;
    Eigen::Quaterniond attitude;
    Eigen::Vector3d linear_velocity;
    Eigen::Vector3d linear_accel;
    double timestamp;

    DroneState()
    {
        position.setZero();
        attitude.setIdentity();
        linear_velocity.setZero();
        linear_accel.setZero();
        timestamp = 0;
    }
};

struct PoseFPR
{
    PoseFPR(const ParamFPR& _param) : param(_param)
    {
        setZero();
    }

    ParamFPR param;
    Eigen::Vector3d platform_position; // platform COM position w.r.t. the reference frame
    Eigen::Quaterniond platform_orientation; // unit quaternion to represent platform's attitude w.r.t. the reference frame
    LegVecType leg_angles; // leg angles w.r.t. the platform plane

    void setPose(const Eigen::Vector3d& pos, const Eigen::Quaterniond& quat, const LegVecType& ang)
    {
        platform_position = pos;
        platform_orientation = quat;
        leg_angles = ang;
    }

    void setPose(const StateVecType& pose)
    {
        platform_position = pose.segment(0, 3);
        platform_orientation = Eigen::Vector4d(pose.segment(3,4));
        leg_angles = pose.segment(7, NUM_LEG);
    }

    void setPose(const PoseFPR& pose)
    {
        *this = pose; 
    }

    StateVecType getPose()
    {
        StateVecType pose;
        pose << platform_position, 
                platform_orientation.vec(), 
                platform_orientation.w(),
                leg_angles;
        return pose;
    }

    void setZero()
    {
        platform_position.setZero();
        platform_orientation.setIdentity();
        leg_angles.setZero();
    }

    void getLegFromDrones(DroneState drone_state[NUM_LEG])
    {
        // angles of revolute joints computed from drones COM positions and the pose of the plaform
        for(int index=0; index<NUM_LEG; index++)
        {
            // ri: vector from COM of platform to revolute joint i expressed in platform frame
            Eigen::Vector3d ri_i = param.pos_ri[index];
            // rotation matrix from platform frame to frame 0
            Eigen::Matrix3d Rp0 = platform_orientation.toRotationMatrix();
            // global position of revolute joint i with respect to frame 0
            Eigen::Vector3d RiPos = platform_position + Rp0*ri_i;

            // unit vector pointing from revolute joint center to drone's center
            Eigen::Vector3d li = drone_state[index].position - RiPos;
            li.normalize();

            // unit vector that the leg is aligned with when angle = 0
            Eigen::Vector3d ui = buildUnitRotMat(param.alpha[index], 'z')*Eigen::Vector3d(1, 0, 0);
            ui = Rp0*ui;

            // computing the angle between two unit vectors
            double cos_ang = li.dot(ui)/(li.norm()*ui.norm());
            leg_angles(index) = acos(cos_ang);
        }
    }
};

struct VelocityFPR
{
    VelocityFPR(ModelFPR& _mdl) : model(&_mdl)
    {
        curr_vel.setZero();
    }

    std::shared_ptr<ModelFPR> model;
    DofVecType curr_vel;

    DofVecType getVelocity()
    {
        return curr_vel;
    }

    void getVelFromDrones(DroneState drone_state[NUM_LEG], StateVecType pose)
    {
        // get jacobian matrix
        model->updateKinematics(pose);
        Eigen::Matrix<double,DIM_ACT,DIM_DOF> J = model->JacobianMat;
        ActVecType vel_drones;
        for (int i=0; i<NUM_LEG; i++)
            vel_drones.segment(i*3,3) = drone_state[i].linear_velocity;

        curr_vel = J.inverse()*vel_drones;
    }
};

struct TrajFPR
{
    StateVecType pose;
    DofVecType velocity;
    DofVecType acceleration;

    TrajFPR()
    {
        pose.setZero();
        velocity.setZero();
        acceleration.setZero();
    }
};

}