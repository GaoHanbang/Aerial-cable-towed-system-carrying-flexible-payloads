#include "common_fpr.h"
#include <sstream>
#include <iomanip>

// Sign function
double sign_func(double v)
{
    if(v!=0) return (v > 0 ? 1 : -1);
    else return 0;
}

// Build a unit rotation matrix where an rotated angle is performed around a given axis
Eigen::Matrix3d buildUnitRotMat(const double angle, const char axis)
{
    Eigen::Matrix3d ret; ret.setZero(); // return

    if(axis == 'Z' || axis == 'z')
    {
        ret = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()); 
        /*ret(0,0) = cos(angle); ret(0,1) = -sin(angle);
        ret(1,0) = sin(angle); ret(1,1) = cos(angle);
        ret(2,2) = 1; */
    }
    else if(axis == 'Y' || axis == 'y')
    {
        ret = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY()); 
        /*ret(0,0) = cos(angle); ret(0,2) = sin(angle);
        ret(1,1) = 1; 
        ret(2,0) = -sin(angle); ret(2,2) = cos(angle);*/
    }
    else if(axis == 'X' || axis == 'x')
    {
        ret = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX());
        /*ret(0,0) = 1; 
        ret(1,1) = cos(angle); ret(1,2) = -sin(angle);
        ret(2,1) = sin(angle); ret(2,2) = cos(angle);*/
    }
    // if char given for axis is wrong, return a zero matrix indicating the error
    
    return ret;
}

// Compute the skew-symmetric matrix corresponding to a given vector
Eigen::Matrix3d skewSymMatrix(Eigen::Vector3d vec)
{
    Eigen::Matrix3d ret;
    ret <<  0, -vec(2), vec(1),
            vec(2), 0, -vec(0),
            -vec(1), vec(0), 0;
    return ret;
}

// Parce eigen vector3d to a string, with default decimal number taken as 2 
std::string parseEigenVec3ToString(Eigen::VectorXd eigen_vec, const int decimal)
{
    std::stringstream ss;
    ss << std::fixed << std::setprecision(decimal);
    int size = int(eigen_vec.size());
    for (int i=0; i < size; i++) 
    {
        if(i==0) ss << "(" << eigen_vec(i) << ", ";
        else if(i==size-1) ss << eigen_vec(i) << ")";
        else ss << eigen_vec(i) << ", ";
    }
    return ss.str();
}