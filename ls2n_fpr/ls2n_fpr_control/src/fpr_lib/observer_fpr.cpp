#include "observer_fpr.h"

namespace FPR
{

WrenchObserverFPR::WrenchObserverFPR(ModelFPR& _mdl, DofVecType _gains) : model(&_mdl)
{
    wrench.setZero();
    wrench_last.setZero();
    integral_term.setZero();

    last_timestamp = -1.;
    is_updated = false;

    gains.setZero();
    for (int i=0; i < DIM_DOF; i++)  
        gains(i,i) = _gains(i);
}

DofVecType WrenchObserverFPR::updateEstimate(const DofVecType& act_force, const StateVecType& q, const DofVecType& dq, double time)
{
    if (last_timestamp == -1.)
    {
        last_timestamp = time;
        return DofVecType::Zero();
    }
    // update dynamics
    model->updateDynamicsCoriolis(q, dq);

    // compute momentum
    DofVecType momentum = model->InertiaMat*dq;

    // compute integral term
    DofVecType variable_temp = act_force + model->CoriolisMat.transpose()*dq - model->GravityVec;
    DofVecType variable_to_integrate = variable_temp + wrench;
    integral_term += variable_to_integrate*(time - last_timestamp);

    // update current estimate of external wrench
    wrench = gains*(momentum - integral_term);

    last_timestamp = time;
    is_updated = true;
    return wrench;
}

DofVecType WrenchObserverFPR::getLastEstimate()
{
    if(is_updated)
    {
        is_updated = false;
        return wrench;
    }
    else 
        return DofVecType::Zero();
}

void WrenchObserverFPR::resetGains(DofVecType _gains)
{
    gains.setZero();
    for (int i=0; i < DIM_DOF; i++)  
        gains(i,i) = _gains(i);
}

}