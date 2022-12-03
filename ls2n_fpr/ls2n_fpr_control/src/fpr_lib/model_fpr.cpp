#include "model_fpr.h"

using namespace FPR;

void ModelFPR::updateKinematics(const StateVecType& current_pos)
{
    computeKinematicModel(current_pos);
}
    
void ModelFPR::updateDynamics(const StateVecType& current_pos, const DofVecType& current_vel)
{
    computeKinematicModel(current_pos);
    computeDynamicModelFast(current_pos, current_vel);
}

void ModelFPR::updateDynamicsCoriolis(const StateVecType& current_pos, const DofVecType& current_vel)
{
    computeKinematicModel(current_pos);
    computeDynamicModelFull(current_pos, current_vel);
}

Eigen::Matrix<double, DIM_ACT, DIM_DOF> ModelFPR::invTransJacobian()
{
    Eigen::Matrix<double, DIM_DOF, DIM_ACT> transJ = JacobianMat.transpose(); // transpose of jacobian matrix
    Eigen::Matrix<double, DIM_ACT, DIM_DOF> invJ; // inverse of Jacobian

    if (DIM_DOF == DIM_ACT) // fully actuated case
    {
        if(abs(transJ.determinant()) < ZERO_THRESHOLD) // dealing with sigularity issue
        {
            #if DEBUG
            std::cout << "[ControllerFPR]: Jacobian matrix not invertible, adding offset to get away from the singular position" << std::endl;
            #endif
            transJ += INV_OFFSET*DofMatType::Identity();
            invJ = transJ.inverse();
        }
        invJ = transJ.inverse();
    }
    else // overactuated case (not used for the moment)
    {
        // proceed pseudo inverse in case of redundancy
        // reference: https://blog.csdn.net/Robot_Starscream/java/article/details/100575556
        auto svd = transJ.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
	    const auto &singularValues = svd.singularValues();
	    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> singularValuesInv(transJ.cols(), transJ.rows());
	    singularValuesInv.setZero();

	    double pinvtoler = ZERO_THRESHOLD; // choose the tolerance
	    for (unsigned int i = 0; i < singularValues.size(); ++i) 
        {
            if (singularValues(i) > pinvtoler)
                singularValuesInv(i,i) = 1.0 / singularValues(i);
            else
                singularValuesInv(i,i) = 0.0;
	    }
	    invJ = svd.matrixV()*singularValuesInv*svd.matrixU().transpose();
        // invJ = transJ.completeOrthogonalDecomposition().pseudoInverse(); 
    }
    invTransJacobianMat = invJ;
    return invJ;
}

// function for computing the Newton-Euler equation at each iteration
DofVecType recursiveNewtonEuler(const double _g, const DofVecType& _vel, const DofVecType& _acc, const ParamFPR _pm, 
    const Eigen::Matrix3d& _Rp, Eigen::Matrix3d* _p_Rleg)
{
    // return of a 9-dimensional vector
    DofVecType ret; ret.setZero();
    
    Eigen::Vector3d acc_p; acc_p << _acc.segment(0,3); // linear acceleration of platform w.r.t frame F0
    Eigen::Vector3d omega_p; omega_p << _vel.segment(3,3); // angular velocity of platform w.r.t frame Fp
    Eigen::Vector3d domega_p; domega_p << _acc.segment(3,3); // angular acceleration of platform w.r.t frame Fp
    
    Eigen::Vector3d acc_leg[NUM_LEG];
    Eigen::Vector3d omega_leg[NUM_LEG]; 
    Eigen::Vector3d domega_leg[NUM_LEG];

    acc_p(2) += _g; // adding gravity
    acc_p = _Rp.transpose()*acc_p; // linear acc of platform w.r.t its frame Fp

    // forward recursive:
    for (int i=0; i < NUM_LEG; i++) // computation of velocity and acceleration of leg frame
    {
        Eigen::Vector3d ri_i = _pm.pos_ri[i];
        Eigen::Matrix3d Rli2p = (*(_p_Rleg+i)).transpose(); // rotation matrix from frame Fli to Fp
        omega_leg[i] = Rli2p*omega_p + Eigen::Vector3d(0,0,_vel(6+i));
        domega_leg[i] = Rli2p*domega_p + Eigen::Vector3d(0,0,_acc(6+i)) + omega_leg[i].cross(Eigen::Vector3d(0,0,_vel(6+i)));
        // angular velocity and acceleration of leg w.r.t its frame Fli
        acc_leg[i] = Rli2p*(acc_p + domega_p.cross(ri_i) + omega_p.cross(omega_p.cross(ri_i)));
        // linear acceleration of leg w.r.t its frame Fli
    }

    // sum of force and torque on each body
    Eigen::Vector3d sum_force_p = _pm.mp*acc_p + domega_p.cross(_pm.ms_p) + omega_p.cross(omega_p.cross(_pm.ms_p));
    Eigen::Vector3d sum_torque_p = _pm.Ip*domega_p + _pm.ms_p.cross(acc_p) + omega_p.cross(_pm.Ip*omega_p);
    
    Eigen::Vector3d sum_force_leg[NUM_LEG];
    Eigen::Vector3d sum_torque_leg[NUM_LEG];
    for (int i=0; i < NUM_LEG; i++)
    {
        sum_force_leg[i] = _pm.m_leg[i]*acc_leg[i] + domega_leg[i].cross(_pm.ms_leg[i]) + omega_leg[i].cross(omega_leg[i].cross(_pm.ms_leg[i]));
        sum_torque_leg[i] = _pm.I_leg[i]*domega_leg[i] + _pm.ms_leg[i].cross(acc_leg[i]) + omega_leg[i].cross(_pm.I_leg[i]*omega_leg[i]);
    }

    Eigen::Vector3d force_p = sum_force_p;
    Eigen::Vector3d torque_p = sum_torque_p;
    // backwards recursive:
    for (int i=0; i < NUM_LEG; i++)
    {   
        Eigen::Vector3d ri_i = _pm.pos_ri[i];
        Eigen::Matrix3d Rli = *(_p_Rleg+i); // rotation matrix from Fp to Fli
        Eigen::Vector3d Fi_p = Rli*sum_force_leg[i]; // force of leg i expressed in frame Fp
        force_p += Fi_p;
        torque_p += Rli*sum_torque_leg[i] + (ri_i).cross(Fi_p);
        ret(6+i) = sum_torque_leg[i](2); // z-axis torque of each leg (around axis of the revolute joint)
    }
    ret.segment(0,3) = _Rp*force_p;
    ret.segment(3,3) = torque_p;

    return ret;
}

/*  Dynamic modeling based on recursive Newton-Euler method, obsolete because of the computation complexity.
Reference: https://link.springer.com/book/10.1007/978-3-319-19788-3. 
*/
void ModelFPR::computeDynamicModelOld(const StateVecType& current_pos, const DofVecType& current_vel)
// Computation of the dynamic model (inertia matrix and coriolis vector) of FPR depending on the current position and veloctiy, the kinematic model is also computed (Jacobian matrix)
{
// frames: inertial reference F0, platform frame Fp, leg i's frame Fli (its origin located at the center of revolute joint)

    Eigen::Quaterniond orient;
    orient = Eigen::Vector4d(current_pos.segment(3,4));
    Eigen::Vector3d qi = current_pos.segment(7,NUM_LEG);

    Rp = orient.toRotationMatrix();

    for (int i=0; i < NUM_LEG; i++)
    {
        Eigen::Vector3d ri_i = param.pos_ri[i];
        Eigen::Matrix3d Rleg_i = buildUnitRotMat(param.alpha[i], 'z')*buildUnitRotMat(-M_PI/2,'x')*buildUnitRotMat(qi(i),'z'); 
        
        // computation of jacobian matrix
        int index = 3*i;
        JacobianMat.block<3,3>(index,0)= Eigen::Matrix3d::Identity();

        Eigen::Vector3d ui_i = Rleg_i*Eigen::Vector3d(param.l_leg,0,0); // leg's tip position expressed in frame Fp
        JacobianMat.block<3,3>(index,3)= -Rp*(skewSymMatrix(ri_i)+skewSymMatrix(ui_i));
        
        Eigen::Vector3d y_li = (Rp*Rleg_i).col(1); // y-axis of the rotation matrix from frame F0 to Fli
        JacobianMat.block<3,1>(index,6+i)= param.l_leg*y_li;

        Rleg[i]= Rleg_i;
    }

    for (int i=0; i<9; i++)
    {
        DofVecType acc; acc.setZero();
        acc(i) = 1;

        // computation of inertia matrix column by column
        InertiaMat.col(i) = recursiveNewtonEuler(0, DofVecType::Zero(), acc, param, Rp, Rleg);
    }

    // computation of coriolis vector
    CoriolisVec = recursiveNewtonEuler(param.gravity, current_vel, DofVecType::Zero(), param, Rp, Rleg);
}


void ModelFPR::computeKinematicModel(const StateVecType& current_pos)
// Computation of kinematic model (Jacobian matrix)
{
    Eigen::Quaterniond orient; 
    orient = Eigen::Vector4d(current_pos.segment(3,4));
    Eigen::Vector3d qi = current_pos.segment(7,NUM_LEG);

    Rp = orient.toRotationMatrix(); // saved to the class member variable to use in the dynamic computation

    for (int i=0; i < NUM_LEG; i++)
    {
        Eigen::Vector3d ri_i = param.pos_ri[i];
        Eigen::Matrix3d Rleg_i = buildUnitRotMat(param.alpha[i], 'z')*buildUnitRotMat(-M_PI/2,'x')*buildUnitRotMat(qi(i),'z'); 
        
        // computation of jacobian matrix
        int index = 3*i;
        JacobianMat.block<3,3>(index,0)= Eigen::Matrix3d::Identity();

        Eigen::Vector3d ui_i = Rleg_i*Eigen::Vector3d(param.l_leg,0,0); // leg's tip position expressed in frame Fp
        JacobianMat.block<3,3>(index,3)= -Rp*(skewSymMatrix(ri_i)+skewSymMatrix(ui_i));
        
        Eigen::Vector3d y_li = (Rp*Rleg_i).col(1); // y-axis of the rotation matrix from frame F0 to Fli
        JacobianMat.block<3,1>(index,6+i)= param.l_leg*y_li;

        Rleg[i]= Rleg_i;
    }
}

/* 
Dynamic modeling based on Recursive Newton-Euler and Composite Rigid Body methods using Spatial Vector algorithm.
    Reference: R. Featherstone, Rigid Body Dynamics Algorithms. Springer, 2008.

Computation of Coriolis Matrix based on Spatial Vector algorithm.
    Reference: S. Echeandia, P. Wensing, Numerical Methods to Compute the Coriolis Matrix and Christoffel Symbols for Rigid-body Systems. 
                Journal of Computational and Nonlinear Dynamics, 2021.
*/
typedef Eigen::Matrix<double,6,6> SpatialMatType;
typedef Eigen::Matrix<double,6,1> SpatialVecType;

SpatialMatType mcI(double mass, Eigen::Vector3d CoM, Eigen::Matrix3d Inertia)
// Rigid-body inertia, inputs: mass, center of mass position and rotational inertia, output: spatial inertia matrix
{
    Eigen::Matrix3d C = skewSymMatrix(CoM);
    SpatialMatType res;
    res.block<3,3>(0,0) = Inertia;
    res.block<3,3>(0,3) = mass*C;
    res.block<3,3>(3,0) = mass*C.transpose();
    res.block<3,3>(3,3) = mass*Eigen::Matrix3d::Identity();
    return res;
}

SpatialMatType rot(double angle, char axis)
// Spatial coordinate transform (rotation), inputs: angle(rad) and rotation axis, output: spatial transformation matrix
{
    Eigen::Matrix3d rotMat = buildUnitRotMat(angle, axis).transpose(); 
    SpatialMatType res; res.setZero();
    res.block<3,3>(0,0) = rotMat;
    res.block<3,3>(3,3) = rotMat;
    return res;
}

SpatialMatType xlt(Eigen::Vector3d r)
// Spatial coordinate transform (translation of origin), input: translation vector, output: spatial transformation matrix
{
    SpatialMatType res; res.setZero();
    res.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
    res.block<3,3>(3,3) = Eigen::Matrix3d::Identity();
    res.block<3,3>(3,0) = -skewSymMatrix(r);
    return res;
}

SpatialMatType crm(SpatialVecType v)
// Spatial cross-product operator (motion), input: velocity vector (angular+linear), output: spatial matrix for cross product
{
    SpatialMatType res; res.setZero();
    Eigen::Vector3d angular = v.segment(0,3); // angular velocity
    Eigen::Vector3d linear = v.segment(3,3); // linear velocity
    Eigen::Matrix3d skew_angular = skewSymMatrix(angular);
    res.block<3,3>(0,0) = skew_angular;
    res.block<3,3>(3,3) = skew_angular;
    res.block<3,3>(3,0) = skewSymMatrix(linear);
    return res;
}

SpatialMatType crf(SpatialVecType v)
// Spatial cross-product operator (force), input: velocity vector (angular+linear), output: spatial matrix for cross product
{
    SpatialMatType res; 
    res = -crm(v).transpose();
    return res;
}

SpatialMatType icrf(SpatialVecType f)
// Spatial cross-product operator for computing coriolis matrix, input: velocity vector (angular+linear), output: spatial matrix
{
    SpatialMatType res; res.setZero();
    Eigen::Matrix3d skew_f1 = -skewSymMatrix(f.segment(0,3));
    Eigen::Matrix3d skew_f2 = -skewSymMatrix(f.segment(3,3));
    res.block<3,3>(0,0) = skew_f1;
    res.block<3,3>(0,3) = skew_f2;
    res.block<3,3>(3,0) = skew_f2;
    return res;
}

SpatialVecType get_gravity(double g)
// Spatial gravitational vector
{
    SpatialVecType res;
    res << 0.0, 0.0, 0.0, 0.0, 0.0, -g;
    return res;
}

// Computation of dynamic model based on Recursive Newton-Euler and Composite Rigid Body methods
void ModelFPR::computeDynamicModelFast(const StateVecType& current_pos, const DofVecType& current_vel)
{
    /// Extracting robot states
    Eigen::Matrix3d Rp_trans = Rp.transpose();
    Eigen::Vector3d vel_p = current_vel.segment(0,3);
    Eigen::Vector3d omega_p = current_vel.segment(3,3);
    Eigen::Vector3d qjoint = current_pos.segment(7,NUM_LEG);
    Eigen::Vector3d qjoint_dot = current_vel.segment(6,NUM_LEG);
    
    SpatialVecType a_grav = get_gravity(param.gravity);

    /// Preparing spatial inertia variables
    SpatialMatType Ip = mcI(param.mp, param.sp, param.Ip);
    SpatialMatType Ileg[NUM_LEG];
    SpatialMatType Ileg_pure; // pure leg inertia matrix, without drone
    Ileg_pure = mcI(param.mleg, param.sleg, param.Ileg);
    SpatialMatType Xtree[NUM_LEG]; // transformation matrix for legs
    for (int i=0; i < NUM_LEG; i++)
    {
        double mass = param.m_leg[i];
        Ileg[i] = mcI(mass, param.ms_leg[i]/mass, param.I_leg[i]);
        Xtree[i] = rot(-M_PI/2, 'x')*rot(param.alpha[i], 'z')*xlt(param.pos_ri[i]);
    }

    /// Forward computation
    // spatial velocity
    SpatialVecType v_p; 
    v_p << omega_p, vel_p; // vp = [omega_p; vel_p]
    // spatial transformation matrix of the platform
    SpatialMatType Xup_p; Xup_p.setZero();
    Xup_p.block<3,3>(0,0) = Rp_trans;
    Xup_p.block<3,3>(3,3) = Rp_trans;
    // spatial joint coordinate transformation 
    SpatialMatType S_p; S_p.setZero();
    S_p.block<3,3>(0,3) = Eigen::Matrix3d::Identity();
    S_p.block<3,3>(3,0) = Rp_trans;
    // spatial acceleration
    SpatialVecType avp_p = Xup_p*-a_grav;
    avp_p.segment(3,3) -= omega_p.cross(vel_p);

    SpatialVecType S_i; // revolute joint angle defined on z axis 
    S_i << 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
    SpatialMatType Xup_i[NUM_LEG]; // spatial transformation matrix of legs
    SpatialVecType v_i[NUM_LEG];
    SpatialVecType avp_i[NUM_LEG];
    SpatialVecType fvp_i[NUM_LEG];
    SpatialVecType fvp_i_pure[NUM_LEG];

    SpatialVecType fvp_p = Ip*avp_p + crf(v_p)*Ip*v_p;
    SpatialVecType fvp_p_pure = fvp_p;
    for (int i=0; i < NUM_LEG; i++)
    {
        Xup_i[i] = rot(qjoint[i],'z')*Xtree[i];
        SpatialVecType vJ = S_i*qjoint_dot[i]; 
        v_i[i] = Xup_i[i]*v_p + vJ;
        avp_i[i] = Xup_i[i]*avp_p + crm(v_i[i])*vJ;
        fvp_i[i] = Ileg[i]*avp_i[i] + crf(v_i[i])*Ileg[i]*v_i[i];
        fvp_i_pure[i] = Ileg_pure*avp_i[i] + crf(v_i[i])*Ileg_pure*v_i[i];
    }
    
    /// Backward computation
    SpatialMatType IC_p = Ip; // composite inertia for the platform
    SpatialMatType IC_p_pure = Ip; // composite inertia for the platform
    CoriolisVec.setZero();
    InertiaMat.setZero();
    CoriolisVec_PassArchi.setZero();
    InertiaMat_PassArchi.setZero();
    for (int i=0; i < NUM_LEG; i++)
    {
        CoriolisVec(6+i) = S_i.transpose()*fvp_i[i];
        fvp_p += Xup_i[i].transpose()*fvp_i[i];

        CoriolisVec_PassArchi(6+i) = S_i.transpose()*fvp_i_pure[i];
        fvp_p_pure += Xup_i[i].transpose()*fvp_i_pure[i];

        // computation of Coriolis Matrix and Inertia Matrix
        SpatialVecType fh = Ileg[i]*S_i;
        InertiaMat(6+i,6+i) = S_i.transpose()*fh;
        SpatialVecType fh_pure = Ileg_pure*S_i;
        InertiaMat_PassArchi(6+i,6+i) = S_i.transpose()*fh_pure;

        fh = Xup_i[i].transpose()*fh;
        SpatialVecType vec = S_p.transpose()*fh;
        InertiaMat.block<6,1>(0,6+i) = vec;
        InertiaMat.block<1,6>(6+i,0) = vec.transpose();

        fh_pure = Xup_i[i].transpose()*fh_pure;
        SpatialVecType vec_pure = S_p.transpose()*fh_pure;
        InertiaMat_PassArchi.block<6,1>(0,6+i) = vec_pure;
        InertiaMat_PassArchi.block<1,6>(6+i,0) = vec_pure.transpose();

        IC_p += Xup_i[i].transpose()*Ileg[i]*Xup_i[i];
        IC_p_pure += Xup_i[i].transpose()*Ileg_pure*Xup_i[i];
    }
    CoriolisVec.segment(0,6) = S_p.transpose()*fvp_p;
    InertiaMat.block<6,6>(0,0) = S_p.transpose()*IC_p*S_p;

    CoriolisVec_PassArchi.segment(0,6) = S_p.transpose()*fvp_p_pure;
    InertiaMat_PassArchi.block<6,6>(0,0) = S_p.transpose()*IC_p_pure*S_p;
}

// Computation of dynamic model (in form of M*ddq + C*dq + G) based on Spatial Vector algorithm
void ModelFPR::computeDynamicModelFull(const StateVecType& current_pos, const DofVecType& current_vel)
{
    /// Extracting robot states
    Eigen::Matrix3d Rp_trans = Rp.transpose();
    Eigen::Vector3d vel_p = current_vel.segment(0,3);
    Eigen::Vector3d omega_p = current_vel.segment(3,3);
    Eigen::Vector3d qjoint = current_pos.segment(7,NUM_LEG);
    Eigen::Vector3d qjoint_dot = current_vel.segment(6,NUM_LEG);
    
    SpatialVecType a_grav = get_gravity(param.gravity);

    /// Preparing spatial inertia variables
    SpatialMatType Ip = mcI(param.mp, param.sp, param.Ip);
    SpatialMatType Ileg[NUM_LEG];
    SpatialMatType Xtree[NUM_LEG]; // transformation matrix for legs
    for (int i=0; i < NUM_LEG; i++)
    {
        double mass = param.m_leg[i];
        Ileg[i] = mcI(mass, param.ms_leg[i]/mass, param.I_leg[i]);
        Xtree[i] = rot(-M_PI/2, 'x')*rot(param.alpha[i], 'z')*xlt(param.pos_ri[i]);
    }

    /// Forward computation
    // spatial velocity
    SpatialVecType v_p; 
    v_p << omega_p, vel_p; // vp = [omega_p; vel_p]
    // spatial transformation matrix of the platform
    SpatialMatType Xup_p; Xup_p.setZero();
    Xup_p.block<3,3>(0,0) = Rp_trans;
    Xup_p.block<3,3>(3,3) = Rp_trans;
    // spatial joint coordinate transformation 
    SpatialMatType S_p; S_p.setZero();
    S_p.block<3,3>(0,3) = Eigen::Matrix3d::Identity();
    S_p.block<3,3>(3,0) = Rp_trans;
    // spatial acceleration
    SpatialVecType avp_p = Xup_p*-a_grav;
    avp_p.segment(3,3) -= omega_p.cross(vel_p);

    SpatialVecType S_i; // revolute joint angle defined on z axis 
    S_i << 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
    SpatialMatType Xup_i[NUM_LEG]; // spatial transformation matrix of legs
    SpatialVecType v_i[NUM_LEG];
    SpatialVecType avp_i[NUM_LEG];
    SpatialVecType fvp_i[NUM_LEG];

    // Variable for computing Coriolis Matrix and Inertia Matrix
    SpatialMatType Sdot_p; Sdot_p.setZero();
    Sdot_p.block<3,3>(3,0) = -skewSymMatrix(omega_p);
    SpatialMatType BC_p = 0.5*(crf(v_p)*Ip + icrf(Ip*v_p) - Ip*crm(v_p));
    SpatialVecType Sdot_i[NUM_LEG];
    SpatialMatType BC_i[NUM_LEG];

    SpatialVecType fvp_p = Ip*avp_p + crf(v_p)*Ip*v_p;
    for (int i=0; i < NUM_LEG; i++)
    {
        Xup_i[i] = rot(qjoint[i],'z')*Xtree[i];
        SpatialVecType vJ = S_i*qjoint_dot[i]; 
        v_i[i] = Xup_i[i]*v_p + vJ;
        avp_i[i] = Xup_i[i]*avp_p + crm(v_i[i])*vJ;
        fvp_i[i] = Ileg[i]*avp_i[i] + crf(v_i[i])*Ileg[i]*v_i[i];

        Sdot_i[i] = crm(v_i[i])*S_i;
        BC_i[i] = 0.5*(crf(v_i[i])*Ileg[i] + icrf(Ileg[i]*v_i[i]) - Ileg[i]*crm(v_i[i]));
    }
    
    /// Backward computation
    SpatialMatType IC_p = Ip; // composite inertia for the platform
    CoriolisVec.setZero();
    CoriolisMat.setZero();
    InertiaMat.setZero();
    for (int i=0; i < NUM_LEG; i++)
    {
        CoriolisVec(6+i) = S_i.transpose()*fvp_i[i];
        fvp_p += Xup_i[i].transpose()*fvp_i[i];

        // computation of Coriolis Matrix and Inertia Matrix
        SpatialVecType f1 = Ileg[i]*Sdot_i[i] + BC_i[i]*S_i;
        SpatialVecType f2 = Ileg[i]*S_i;
        SpatialVecType f3 = BC_i[i].transpose()*S_i;
        CoriolisMat(6+i,6+i) = S_i.transpose()*f1;
        InertiaMat(6+i,6+i) = S_i.transpose()*f2;

        f1 = Xup_i[i].transpose()*f1;
        f2 = Xup_i[i].transpose()*f2;
        f3 = Xup_i[i].transpose()*f3;
        CoriolisMat.block<6,1>(0,6+i) = S_p.transpose()*f1;
        CoriolisMat.block<1,6>(6+i,0) = (Sdot_p.transpose()*f2 + S_p.transpose()*f3).transpose();

        SpatialVecType vec = S_p.transpose()*f2;
        InertiaMat.block<6,1>(0,6+i) = vec;
        InertiaMat.block<1,6>(6+i,0) = vec.transpose();

        IC_p += Xup_i[i].transpose()*Ileg[i]*Xup_i[i];
        BC_p += Xup_i[i].transpose()*BC_i[i]*Xup_i[i];
    }
    CoriolisVec.segment(0,6) = S_p.transpose()*fvp_p;
    CoriolisMat.block<6,6>(0,0) = S_p.transpose()*(IC_p*Sdot_p + BC_p*S_p);
    InertiaMat.block<6,6>(0,0) = S_p.transpose()*IC_p*S_p;
    // computing gravity term if gravity is set
    if (param.gravity != 0.0)
        computeGravityTerm(current_pos);
}

void ModelFPR::computeGravityTerm(const StateVecType& current_pos)
{
    GravityVec.setZero();
    /// Extracting robot states
    Eigen::Vector3d qjoint = current_pos.segment(7,NUM_LEG);

    Eigen::Vector3d vec_g; // gravity vector
    vec_g << 0.0, 0.0, param.gravity;

    Eigen::RowVector3d axis; //joint axis
    axis << 0, 0, 1;

    double mass_tot = param.mp;
    for(int i=0; i < NUM_LEG; i++)
    {
        mass_tot += param.m_leg[i];

        Eigen::Matrix3d Rleg_i = buildUnitRotMat(param.alpha[i], 'z')*buildUnitRotMat(-M_PI/2,'x')*buildUnitRotMat(qjoint(i),'z');  
        Eigen::Vector3d ms_i = Rp*Rleg_i*param.ms_leg[i];
        GravityVec.segment(3,3) += ms_i.cross(vec_g);

        Eigen::Vector3d vec_g_leg = Rleg_i.transpose()*Rp.transpose()*vec_g;
        GravityVec(6+i) = axis*param.ms_leg[i].cross(vec_g_leg);
    }
    GravityVec.segment(0,3) = mass_tot*vec_g;
    GravityVec.segment(3,3) += param.ms_p.cross(vec_g);
    GravityVec.segment(3,3) = Rp.transpose()*GravityVec.segment(3,3);
}