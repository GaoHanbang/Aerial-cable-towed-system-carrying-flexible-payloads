#include "param_fpr.h"

using namespace FPR;

// Compute dynamic parameters, adding drone masses effects in the moment of inertia and the COM position of legs
void ParamFPR::computeParam()
{
    Eigen::Vector3d s_drones;
    s_drones << l_leg, 0, 0; // COM position of drones w.r.t leg's frame

    Eigen::Matrix3d s_square_drones;
    s_square_drones << 0, 0, 0,
                       0, l_leg*l_leg, 0,
                       0, 0, l_leg*l_leg;

    ms_p = mp*sp; // computation of first moment of inertia

    //Mirroring the inertia matrices
    Ip(1, 0) = Ip(0, 1);
    Ip(2, 0) = Ip(0, 2);
    Ip(2, 1) = Ip(1, 2);
    Ileg(1, 0) = Ileg(0, 1);
    Ileg(2, 0) = Ileg(0, 2);
    Ileg(2, 1) = Ileg(1, 2);

    for (int i=0; i < NUM_LEG; i++)
    {
        m_leg[i] = mleg + m_drones[i]; // mass
        ms_leg[i] = mleg*sleg + m_drones[i]*s_drones; // first moment
        I_leg[i] = Ileg + m_drones[i]*s_square_drones; // moment of inertia

        Eigen::Matrix3d Rz_alpha = buildUnitRotMat(alpha[i], 'z');
        pos_ri[i] = Rz_alpha*Eigen::Vector3d(r_p, 0, 0);

        beta[i] *= M_PI/180; // converting angle radius to degrees 
    }
}

// Print the parameters values onto the given ostream
void ParamFPR::printParam(std::ostream *pOut=NULL)
{
    if(pOut == NULL) return;
    for (int i=0; i<NUM_LEG; i++)
    {   
        int index = i+1;
        char msg_temp[100];
        sprintf(msg_temp, "drone_param/drone%d/angle: %.3f", index, beta[i]);
        *pOut << msg_temp << std::endl;
        sprintf(msg_temp, "drone_param/drone%d/mass: %.3f", index, m_drones[i]);
        *pOut << msg_temp << std::endl;
    }
    *pOut << "geometric_param/l_leg: " << l_leg << std::endl;
    *pOut << "geometric_param/r_p: " << r_p << std::endl;
    *pOut << "dynamic_param/mass_p: " << mp << std::endl;
    *pOut << "dynamic_param/mass_leg: " << mleg << std::endl;
    *pOut << "dynamic_param/s_p: " << sp << std::endl;
    *pOut << "dynamic_param/s_leg: " << sleg << std::endl;
    *pOut << "dynamic_param/I_p: " << Ip << std::endl;
    *pOut << "dynamic_param/I_leg: " << Ileg << std::endl;
    *pOut << "dynamic_param/mass_drone1: " << m_drones[0] << std::endl;
    *pOut << "dynamic_param/mass_drone2: " << m_drones[1] << std::endl;
    *pOut << "dynamic_param/mass_drone3: " << m_drones[2] << std::endl;

    std::string axis[3] = {"x","y","z"};
    for(int i=0; i<3; i++)
    {
        char msg_temp[100];
        sprintf(msg_temp, "control_gain/position/kp_%s: %f", axis[i].data(), ctrl_gain.kp_p[i]);
        *pOut << msg_temp << std::endl;
        sprintf(msg_temp, "control_gain/position/kd_%s: %f", axis[i].data(), ctrl_gain.kd_p[i]);
        *pOut << msg_temp << std::endl;
        sprintf(msg_temp, "control_gain/position/ki_%s: %f", axis[i].data(), ctrl_gain.ki_p[i]);
        *pOut << msg_temp << std::endl;
        sprintf(msg_temp, "control_gain/orientation/kp_%s: %f", axis[i].data(), ctrl_gain.kp_o[i]);
        *pOut << msg_temp << std::endl;
        sprintf(msg_temp, "control_gain/orientation/kd_%s: %f", axis[i].data(), ctrl_gain.kd_o[i]);
        *pOut << msg_temp << std::endl;
        sprintf(msg_temp, "control_gain/orientation/ki_%s: %f", axis[i].data(), ctrl_gain.ki_o[i]);
        *pOut << msg_temp << std::endl;
    }
    *pOut << "control_gain/leg_angle/kp: " << ctrl_gain.kp_l << std::endl;
    *pOut << "control_gain/leg_angle/kd: " << ctrl_gain.kd_l << std::endl;
    *pOut << "control_gain/leg_angle/ki: " << ctrl_gain.ki_l << std::endl;
    *pOut << "control_gain/impedance/mv_p: " << ctrl_gain.mv_p << std::endl;
    *pOut << "control_gain/impedance/dv_p: " << ctrl_gain.dv_p << std::endl;
    *pOut << "control_gain/impedance/kv_p: " << ctrl_gain.kv_p << std::endl;
    *pOut << "control_gain/impedance/mv_o: " << ctrl_gain.mv_o << std::endl;
    *pOut << "control_gain/impedance/dv_o: " << ctrl_gain.dv_o << std::endl;
    *pOut << "control_gain/impedance/kv_o: " << ctrl_gain.kv_o << std::endl;
    *pOut << "control_gain/impedance/mv_l: " << ctrl_gain.mv_l << std::endl;
    *pOut << "control_gain/impedance/dv_l: " << ctrl_gain.dv_l << std::endl;
    *pOut << "control_gain/impedance/kv_l: " << ctrl_gain.kv_l << std::endl;
    *pOut << "observer_gain/kp: " << observer_gain.k_p << std::endl;
    *pOut << "observer_gain/ko: " << observer_gain.k_o << std::endl;
    *pOut << "observer_gain/kl: " << observer_gain.k_l << std::endl;
}