/** \file common_types.hpp
*  \brief Data types used throughout the code.
*
* Data structures and type definitions used are defined here.
*/

#ifndef COMMON_TYPE_HPP
#define COMMON_TYPE_HPP



// external libraries
#include "Eigen/Dense"


// common eigen matrix definition used throughout the code
typedef Eigen::Matrix<double, 2, 1> Vector2;
typedef Eigen::Matrix<double, 3, 1> Vector3;
typedef Eigen::Matrix<double, 4, 1> Vector4;
typedef Eigen::Matrix<double, 3, 3> Matrix3;

namespace fdcl
{

struct state_t
{
    Vector3 x = Vector3::Zero();
    Vector3 v = Vector3::Zero();
    Vector3 a = Vector3::Zero();
    Matrix3 R = Matrix3::Identity();
    Vector3 W = Vector3::Zero();
};


/** \brief Data structure to store command (or desired) values
 */
class command_t
{
public:
    Vector3 Wd = Vector3::Zero();
    Vector3 Wd_dot = Vector3::Zero();
    Vector3 Wd_2dot = Vector3::Zero();
    Matrix3 Rd = Matrix3::Identity();
    Vector3 xd = Vector3::Zero();
    Vector3 xd_dot = Vector3::Zero();
    Vector3 xd_2dot = Vector3::Zero();
    Vector3 xd_3dot = Vector3::Zero();
    Vector3 xd_4dot = Vector3::Zero();
    Vector3 b1d = Vector3::Zero();
    Vector3 b1d_dot = Vector3::Zero();
    Vector3 b1d_ddot = Vector3::Zero();
    Vector3 b3d = Vector3::Zero();
    Vector3 b3d_dot = Vector3::Zero();
    Vector3 b3d_ddot = Vector3::Zero();
    Vector3 b1c = Vector3::Zero();
    double wc3 = 0.0;
    double wc3_dot = 0.0;

    Vector3 b1 = Vector3::Zero();
    Vector3 b2 = Vector3::Zero();
    Vector3 b3 = Vector3::Zero();
};

}  // end of namespace fdcl

#endif
