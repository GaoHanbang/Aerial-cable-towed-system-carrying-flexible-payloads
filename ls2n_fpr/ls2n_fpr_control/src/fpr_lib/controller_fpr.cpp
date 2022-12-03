#include "controller_fpr.h"

using namespace FPR;

#define INTEGRAL_LIMIT 5

ControllerFPR::ControllerFPR(ModelFPR& _mdl, ControllerType ctrlType) : model(&_mdl), controller_type(ctrlType)                                
{  
    traj_updated = false;
    pose_updated = false;
    vel_updated = false;

    pose.setZero();
    des_pose.setZero();

    /* initialize PID control gains */
    for (int i=0; i<3; i++)
    {
        // proportional term
        control_gains.PID_Kp(i,i) = model->param.ctrl_gain.kp_p[i];
        control_gains.PID_Kp(i+3,i+3) = model->param.ctrl_gain.kp_o[i];
        // derivative term
        control_gains.PID_Kd(i,i) = model->param.ctrl_gain.kd_p[i];
        control_gains.PID_Kd(i+3,i+3) = model->param.ctrl_gain.kd_o[i];
        // integral term
        control_gains.PID_Ki(i,i) = model->param.ctrl_gain.ki_p[i];
        control_gains.PID_Ki(i+3,i+3) = model->param.ctrl_gain.ki_o[i];
    }
    control_gains.PID_Kp.block<NUM_LEG,NUM_LEG>(6,6) = model->param.ctrl_gain.kp_l*Eigen::Matrix<double,NUM_LEG,NUM_LEG>::Identity();
    control_gains.PID_Kd.block<NUM_LEG,NUM_LEG>(6,6) = model->param.ctrl_gain.kd_l*Eigen::Matrix<double,NUM_LEG,NUM_LEG>::Identity();
    control_gains.PID_Ki.block<NUM_LEG,NUM_LEG>(6,6) = model->param.ctrl_gain.ki_l*Eigen::Matrix<double,NUM_LEG,NUM_LEG>::Identity();
    
    /* initialize impedance gains */
    control_gains.Mv.block<3,3>(0,0) = model->param.ctrl_gain.mv_p*Eigen::Matrix3d::Identity();
    control_gains.Mv.block<3,3>(3,3) = model->param.ctrl_gain.mv_o*Eigen::Matrix3d::Identity();
    control_gains.Dv.block<3,3>(0,0) = model->param.ctrl_gain.dv_p*Eigen::Matrix3d::Identity();
    control_gains.Dv.block<3,3>(3,3) = model->param.ctrl_gain.dv_o*Eigen::Matrix3d::Identity();
    control_gains.Kv.block<3,3>(0,0) = model->param.ctrl_gain.kv_p*Eigen::Matrix3d::Identity();
    control_gains.Kv.block<3,3>(3,3) = model->param.ctrl_gain.kv_o*Eigen::Matrix3d::Identity();
    control_gains.Mv.block<NUM_LEG,NUM_LEG>(6,6) = model->param.ctrl_gain.mv_l*Eigen::Matrix<double,NUM_LEG,NUM_LEG>::Identity();
    control_gains.Dv.block<NUM_LEG,NUM_LEG>(6,6) = model->param.ctrl_gain.dv_l*Eigen::Matrix<double,NUM_LEG,NUM_LEG>::Identity();
    control_gains.Kv.block<NUM_LEG,NUM_LEG>(6,6) = model->param.ctrl_gain.kv_l*Eigen::Matrix<double,NUM_LEG,NUM_LEG>::Identity();

    /* initialize control error and external wrench */
    control_error.pose_error = DofVecType::Zero();
    control_error.accum_error = DofVecType::Zero();
    control_error.velocity_error = DofVecType::Zero();
    external_wrench.desired = DofVecType::Zero();
    external_wrench.estimated = DofVecType::Zero();
}

void ControllerFPR::updateTrajectory(const StateVecType& _des_pose, const DofVecType& _des_vel, const DofVecType& _des_acc)
{
    des_pose = _des_pose;

    des_velocity = _des_vel;
    des_acceleration = _des_acc;

    traj_updated = true;
}

void ControllerFPR::updateCurrentPose(const StateVecType& _pose)
{
    pose = _pose;
    pose_updated = true;
}

void ControllerFPR::updateCurrentVelocity(const DofVecType& _vel)
{
    velocity = _vel;
    vel_updated = true;
}

void ControllerFPR::getDroneTargets(std::vector<DroneTarget>& _target)
{
    _target.clear();
    for (int i=0; i<NUM_LEG; i++)
    {
        _target.push_back(drone_target[i]);
    }
}
 
bool ControllerFPR::spinController(std::string& msgerr)
{
    if(!pose_updated || !vel_updated)
    {
        msgerr = "Current pose or velocity not updated";
        return false;
    }

    if(!traj_updated)
    {
        msgerr = "Desired trajectory not updated";
        return false;
    }

    model->updateDynamics(pose, velocity);

    if(controller_type == ControllerType::IMPEDANCE)
        doImpedanceControl();
    else
        doMotionControl();

    computeDroneTarget();

    // reset the flags for the next control step
    traj_updated = false;
    pose_updated = false;
    vel_updated = false;
    return true;
}

void ControllerFPR::doMotionControl()
{
    if (controller_type == PD_CTC)
        aux_control_output = computePDControlLaw();
    else if (controller_type == PID_CTC)
        aux_control_output = computePIDControlLaw();

    Eigen::Matrix<double, DIM_ACT, DIM_DOF> invJ = model->invTransJacobian(); // inverse of transpose of Jacobian

    actuation_wrench = model->InertiaMat*aux_control_output + model->CoriolisVec;
    thrust_force = invJ*actuation_wrench;
    interaction_force = invJ*(model->InertiaMat_PassArchi*aux_control_output + model->CoriolisVec_PassArchi);
}

void ControllerFPR::doImpedanceControl()
{
    // tracking error
    DofVecType err = computeControlError();

    // velocity error
    DofVecType err_vel = des_velocity - velocity;

    // external force error
    DofVecType err_wrench = -external_wrench.estimated - external_wrench.desired;  
    // err_wrench = (-external) - desired
    // external wrench is along negative direction of the desired wrench, since the former is acting on the robot, the later is exerting on the environment

    // impedance control law
    aux_control_output = des_acceleration + control_gains.Mv.inverse()*
        (control_gains.Kv*err + control_gains.Dv*err_vel - err_wrench);

    Eigen::Matrix<double, DIM_ACT, DIM_DOF> invJ = model->invTransJacobian(); // inverse of transpose of Jacobian

    actuation_wrench = model->InertiaMat*aux_control_output + model->CoriolisVec - external_wrench.estimated;
    thrust_force = invJ*actuation_wrench;
    interaction_force = invJ*(model->InertiaMat_PassArchi*aux_control_output + model->CoriolisVec_PassArchi);
}

DofVecType ControllerFPR::computeControlError()
{
    DofVecType err; err.setZero();
    err.segment(0,3) = des_pose.segment(0,3) - pose.segment(0,3);
    err.segment(6,NUM_LEG) = des_pose.segment(7,NUM_LEG) - pose.segment(7,NUM_LEG);

    Eigen::Quaterniond des_quaternion; des_quaternion = Eigen::Vector4d(des_pose.segment(3,4)); // desired quaternion
    Eigen::Quaterniond act_quaternion; act_quaternion = Eigen::Vector4d(pose.segment(3,4)); // actual quaternion
    Eigen::Quaterniond quat_err = act_quaternion.conjugate()*des_quaternion;
    err.segment(3,3) = sign_func(quat_err.w())*quat_err.vec();

    return err;
}

DofVecType ControllerFPR::computePDControlLaw()
{
    // tracking error
    DofVecType err = computeControlError();

    // auxilary control input based on PD controller
    DofVecType err_vel = des_velocity - velocity;
    DofVecType u = des_acceleration + control_gains.PID_Kp*err + control_gains.PID_Kd*err_vel;
    
    // update control errors
    control_error.pose_error = err;
    control_error.velocity_error = err_vel;
    control_error.accum_error += err;

    return u;
}

DofVecType ControllerFPR::computePIDControlLaw()
{
    // tracking error on the coordinates
    DofVecType err = computeControlError();
    control_error.pose_error = err; // update pose error

    // velocity error: difference between desired and actual velocity
    DofVecType err_vel = des_velocity - velocity;
    control_error.velocity_error = err_vel; // update velocity error
    
    // update accumulate error
    control_error.accum_error += err;

    bool integral_switch = true;
    for(int i=0; i<DIM_DOF; i++)
    {
        if(err(i) > INTEGRAL_LIMIT)
            integral_switch = false;
    }
    
    // compute acceleration control output
    DofVecType u;
    if(integral_switch)
    {
        u = des_acceleration + control_gains.PID_Kp*err + control_gains.PID_Ki*control_error.accum_error 
            + control_gains.PID_Kd*err_vel;
    }
    else
        u = des_acceleration + control_gains.PID_Kp*err + control_gains.PID_Kd*err_vel;
    return u;
}

// Computation of drones targets represented by desired thrusts and attitudes of drones, from actuation forces
void ControllerFPR::computeDroneTarget()
{
    Eigen::Vector3d dir_leg; // leg's direction
    Eigen::Vector3d si_0; // position of spherical joint (leg's tip) expressed in frame F0
    Eigen::Vector3d ri_0; // position of revolute joint (leg frame's origin) expressed in frame F0
    
    Eigen::Matrix3d Rp = model->Rp;

    for (int i=0; i < NUM_LEG; i++)
    {   
        // compute leg's direction vector
        ri_0 = Rp*model->param.pos_ri[i];
        si_0 = ri_0 + Rp*model->Rleg[i]*Eigen::Vector3d(model->param.l_leg, 0, 0);
        dir_leg = si_0 - ri_0; // direction of leg: revolute joint towards spherical joint

        dir_leg.normalize(); // normalize the vector

        // construct rotation matrix to represent drone's attitude
        Eigen::Vector3d fi = thrust_force.segment(3*i, 3);
        double norm_fi = fi.norm(); // desired thrust 
        
        Eigen::Vector3d xAxis, yAxis, zAxis; // x,y,z axis of rotation matrix for drone's attitude
        zAxis = fi/norm_fi; // z axis of desired drone's attitude
        xAxis = dir_leg.cross(zAxis); // x axis being orthogonal to leg's direction and z axis

        if(xAxis.norm() < ZERO_THRESHOLD) // configuration where leg's direction is aligned with z axis
        {
            double angle = model->param.alpha[i] - (M_PI+model->param.beta[i]); // angle between x axis of MAVROS frame and that of frame Fp
            xAxis = Eigen::Vector3d(cos(angle), sin(angle), 0);
            // x axis of drone expressed in frame F0 before rotating the revolute joint

            Eigen::Vector3d rAxis = Rp*model->Rleg[i]*Eigen::Vector3d(0,0,1);
            // axis of revolute joint for the leg i
            xAxis = Eigen::AngleAxisd(pose(7+i), rAxis)*xAxis;
        }
        else
        {
            xAxis = Eigen::AngleAxisd(model->param.beta[i], zAxis)*xAxis; 
            // rotate a constant angle around z axis to be coherent with MAVROS frame 
        }
        xAxis.normalize();
        yAxis = zAxis.cross(xAxis); 

        Eigen::Matrix3d RotDi; // rotation matrix representing the attitude of drone i
        RotDi << xAxis, yAxis, zAxis;

        // update drone targets
        drone_target[i].thrust = norm_fi;
        drone_target[i].attitude = Eigen::Quaterniond(RotDi);    
    }
}

void ControllerFPR::updateExternalWrench(const DofVecType& estimated)
{
    external_wrench.estimated = estimated;
}

void ControllerFPR::updateDesiredWrench(const DofVecType& desired)
{
    external_wrench.desired = desired;
}