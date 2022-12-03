#pragma once

#include <vector>
#include <memory>
#include <eigen3/Eigen/Dense>

//// Definition of constants
#ifndef M_PI
#define M_PI    3.1415926
#endif

#define GRAVITY   9.81

//// Constants relating to FPR project
#define NUM_LEG  3  // number of legs (= number of drones)
#define DIM_ACT  3*NUM_LEG  // dimension of actuators
#define DIM_DOF  6+NUM_LEG  // dimension of robot dof
// In current version, NUM_LEG is 3, DIM_ACT = DIM_DOF: fully actuated => Matrices are of dimension 9x9

//// Constants for math operation
#define ZERO_THRESHOLD 1E-7  // zero threshold value
#define INV_OFFSET 1E-4      // offset to be added for the inversion of singular Jacobian

//// Customized type definition
typedef Eigen::Matrix<double, NUM_LEG, 1> LegVecType; // Eigen vector of which the dimension correspoinding to number of legs
typedef Eigen::Matrix<double, DIM_DOF+1, 1> StateVecType; // Eigen vector for FPR pose state (Dimension DIM_DOF+1, since orientation is represented by quaternion)
typedef Eigen::Matrix<double, DIM_DOF, 1> DofVecType; // Eigen vector for FPR states that have dimension of robot's dof (velocity, acceleration variables)
typedef Eigen::Matrix<double, DIM_DOF, DIM_DOF> DofMatType; // Eigen matrix that has dimension of robot's dof (inertia matrix, jacobian matrix)
typedef Eigen::Matrix<double, DIM_ACT, 1> ActVecType;

///////////////// Utility Functions ////////////////////////

double sign_func(double v);

Eigen::Matrix3d buildUnitRotMat(const double angle, const char axis);

Eigen::Matrix3d skewSymMatrix(Eigen::Vector3d vec);

std::string parseEigenVec3ToString(Eigen::VectorXd eigen_vec, const int decimal = 2);