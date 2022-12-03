#pragma once

#include <eigen3/Eigen/Dense>
#include "model_fpr.h"
#include <fstream>

namespace FPR{

class WrenchObserverFPR
// Class for external wrench observer of the FPR based on momentum approach
{
public:

    WrenchObserverFPR(ModelFPR& _mdl, DofVecType _gains);
    ~WrenchObserverFPR(){};

    DofVecType getLastEstimate();

    DofVecType updateEstimate(const DofVecType& act_force, const StateVecType& q, const DofVecType& dq, double time);

    void resetGains(DofVecType _gains);

private:

    std::shared_ptr<ModelFPR> model;
    DofVecType wrench;
    DofVecType wrench_last;
    DofVecType integral_term;
    DofMatType gains;

    double last_timestamp;
    bool is_updated;
};

}