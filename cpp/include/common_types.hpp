/** \file common_types.hpp
*  \brief Data types used throughout the code.
*
* Data structures and type definitions used are defined here.
*/

/*
 * Copyright (c) 2020 Flight Dynamics and Control Lab
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in 
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
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

/** \brief Data structure to store current states.
 */
struct state_t
{
    Vector3 x = Vector3::Zero(); /**< Position */
    Vector3 v = Vector3::Zero(); /**< Velocity */
    Vector3 a = Vector3::Zero(); /**< Acceleration */
    Matrix3 R = Matrix3::Identity(); /**< Attitude in SO(3) */
    Vector3 W = Vector3::Zero(); /**< Body angular velocity */
};


/** \brief Data structure to store command (or desired) values.
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
