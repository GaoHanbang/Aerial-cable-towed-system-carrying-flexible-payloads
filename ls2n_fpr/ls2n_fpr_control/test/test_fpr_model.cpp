// @brief: GTest-built testing functions for the modeling of Fpr.
// Results are compared with the outputs from Matlab.

#include <iostream>
#include <gtest/gtest.h>
#include <eigen3/Eigen/Dense>

#include "model_fpr.h"

using namespace FPR;

ParamFPR param_G;

void setDefaultParameters(ParamFPR& param)
{
    param.l_leg = 1.043;
    param.r_p = 0.117;
    param.mp = 0.212;
    param.sp(2) = 3.052e-3;
    param.mleg = 0.072;
    param.sleg(0) = 0.599;
    param.Ip(0,0) = 5.427e-4;
    param.Ip(1,1) = 5.427e-4;
    param.Ip(2,2) = 0.001;
    param.Ileg(0,0) = 1.5707e-06;
    param.Ileg(1,1) = 0.035;
    param.Ileg(2,2) = 0.035;
    param.Ileg(0,1) = 2.128e-06;
    param.Ileg(0,2) = 2.730e-19;
    param.Ileg(1,2) = -3.0723e-23;
    param.m_drones[0] = 1.066;
    param.m_drones[1] = 1.066;
    param.m_drones[2] = 1.066;
    param.beta[0] = -45; // in degrees
    param.beta[1] = -45;
    param.beta[2] = -45;
    param.gravity = 9.81;

    param.computeParam();
    return;
}

Eigen::Matrix<double, 10, 1> q;
Eigen::Matrix<double, 9, 1> dq;
ModelFPR* pModelFpr = NULL;

void computeModel()
{
    q << 0.814723686393179, 0.905791937075619, 0.126986816293506,
        0.550146179873698, 0.084859170659822, 0.242290647485403, 0.794627802223411,
        0.546881519204984, 0.957506835434298, 0.964888535199277;
    
    dq << 0.157613081677548, 0.970592781760616, 0.957166948242946, 0.485375648722841, 
        0.800280468888800, 0.141886338627215, 0.421761282626275, 0.915735525189067, 0.792207329559554;
    
    pModelFpr = new ModelFPR(param_G);
    pModelFpr->updateDynamics(q,dq);
}


TEST(TestFprModeling, test_kinematic_model)
{
    std::cout << "[----------] Testing Jacobian Matrix" << std::endl;
    
    Eigen::Matrix<double, 9, 9> Jacobian_Truth;
    Jacobian_Truth << 1, 0, 0, -0.158209947526348, -0.875510708577561, -0.293990062904572, -0.828540653660888, 0, 0,
                    0, 1, 0, 0.150387082108243, 0.580271159615366, 0.279453399867126, 0.482786556875332, 0, 0,
                    0, 0, 1, 0.496525174244728, -0.454719723189001, 0.922656694425623, -0.410227407338480, 0, 0,
                    1, 0, 0, 0.000595621484986736, -0.596514379462277, -0.434709187596091, 0, 0.344711421310774, 0,
                    0, 1, 0, -0.281103007132086, -0.706897107173627, -0.396649589383283, 0, 0.499406296103365, 0,
                    0, 0, 1, 1.01703388082699, 0.0240343999542694, -0.410158763035978, 0, -0.848302061432262, 0,
                    1, 0, 0, -0.497268100306703, -0.601608428919256, 0.638275904581233, 0, 0, -0.0828725379809315,
                    0, 1, 0, 0.750748245906646, -0.706376330251097, 0.196021657435031, 0, 0, 0.905870615187277,
                    0, 0, 1, 0.550686460233781, 0.0222535286813498, -0.244328951614815, 0, 0, 0.510274015592430;

    Eigen::Matrix<double, 9, 9> Jacobian = pModelFpr->JacobianMat;

    ASSERT_TRUE(Jacobian.isApprox(Jacobian_Truth, 1E-7));
    std::cout << "[----------] Jacobian Matrix is Correct" << std::endl;
}

TEST(TestFprModeling, test_inertia_matrix)
{
    std::cout << "[----------] Testing Inertia Matrix" << std::endl;
    
    Eigen::Matrix<double, 9, 9> InertiaMatrix_Truth;
    InertiaMatrix_Truth << 3.626, 0, 0,-0.724995291780381, -2.29567623691366, -0.100130287769109, -0.917484453112273, 0.381716175863489, -0.0917689009603878,
                        0, 3.626, 0, 0.686413361208367, -0.922115475628578, 0.0872873781211801, 0.534613670611769, 0.553017538049203, 1.00311578230143, 
                        0, 0, 3.626, 2.28524985813478, -0.452191697177257, 0.296956906898002, -0.454265299850522, -0.939367247060569, 0.565051906704435,
                        -0.724995291780381, 0.686413361208367, 2.28524985813478, 2.77256332689803, 0.00429315139524285, -0.0706471841149956, 0, -1.10195074932930, 1.10124249334591,
                        -2.29567623691366, -0.922115475628578, -0.452191697177257, 0.00429315139524280, 3.32864347009911, 0.00213555855878735, 1.31006924564774, -0.636211561758980, -0.635802649976316,
                        -0.100130287769110, 0.0872873781211801, 0.296956906898003, -0.0706471841149955, 0.00213555855878735, 2.24286027957618, 0, 0, 0,
                        -0.917484453112273, 0.534613670611769, -0.454265299850522, 0, 1.31006924564774, 0, 1.194647034, 0, 0,
                        0.381716175863489, 0.553017538049203, -0.939367247060569, -1.10195074932930, -0.636211561758980, 0, 0, 1.194647034, 0,
                        -0.0917689009603878, 1.00311578230143, 0.565051906704435, 1.10124249334591, -0.635802649976316, 0, 0, 0, 1.194647034;
    
    Eigen::Matrix<double, 9, 9> InertiaMatrix_Featherstone = pModelFpr->InertiaMat;

    ASSERT_TRUE(InertiaMatrix_Featherstone.isApprox(InertiaMatrix_Truth, 1E-7)); // cross validation between two methods
    std::cout << "[----------] Inertia Matrix is Correct" << std::endl;
}

TEST(TestFprModeling, test_coriolis_vector)
{
    std::cout << "[----------] Testing Coriolis Vector" << std::endl;
    
    Eigen::Matrix<double, 9, 1> CoriolisVector_Truth;
    CoriolisVector_Truth << -1.498271806372737, -3.090561688475971, 37.531282549683375, 22.697386362863707, 
        -3.347553157076804, 2.844255867493057, -4.569976916996424, -9.220517658749985, 5.021499751781042;

    Eigen::Matrix<double, 9, 1> CoriolisVector_Featherstone = pModelFpr->CoriolisVec;

    ASSERT_TRUE(CoriolisVector_Featherstone.isApprox(CoriolisVector_Truth, 1E-7));
    std::cout << "[----------] Coriolis Vector is Correct" << std::endl;
}

TEST(TestFprModeling, test_coriolis_matrix)
{
    std::cout << "[----------] Testing Coriolis Matrix" << std::endl;
    
    pModelFpr->param.gravity = 0.0; // reset gravity

    pModelFpr->updateDynamicsCoriolis(q,dq);

    Eigen::Matrix<double, 9, 1> CoriolisVector_Computed = pModelFpr->CoriolisMat*dq;
    Eigen::Matrix<double, 9, 1> CoriolisVector = pModelFpr->CoriolisVec;

    ASSERT_TRUE(CoriolisVector_Computed.isApprox(CoriolisVector, 1E-7));
    std::cout << "[----------] Coriolis Matrix is Correct" << std::endl;
}

TEST(TestFprModeling, test_gravity_term)
{
    std::cout << "[----------] Testing Gravity Term" << std::endl;

    Eigen::Matrix<double, 9, 1> GravityVector_Truth;
    GravityVector_Truth << 0, 0, 35.5710600000000, 22.4241117114051, -4.43683666484762, 
    2.91314725666941, -4.45634259153362, -9.21519269366418, 5.54315920477051;

    pModelFpr->param.gravity = 9.81; // reset gravity to 9.81
    pModelFpr->updateDynamicsCoriolis(q,dq);
    Eigen::Matrix<double, 9, 1> GravityVector_Computed = pModelFpr->GravityVec;
    
    ASSERT_TRUE(GravityVector_Computed.isApprox(GravityVector_Truth, 1E-7));
    std::cout << "[----------] Gravity Vector is Correct" << std::endl;
}


int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    setDefaultParameters(param_G);
    computeModel();

    return RUN_ALL_TESTS();
    delete pModelFpr;
    return 0;
}