/** \file matrix_utils.hpp
*  \brief Miscellaneous math functions
*
*  Miscellaneous math functions used in FDCL are defined here
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

#ifndef FDCL_MATRIX_UTILS_HPP
#define FDCL_MATRIX_UTILS_HPP

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include "common_types.hpp"


/** \fn Matrix3 hat(const Vector3 v)
* Returns the hat map of a given 3x1 vector. This is the inverse of vee map.
* @param v vector which the hat map is needed to be operated on
* @return hat map of the input vector
*/
Matrix3 hat(const Vector3 v);


/** \fn Vector3 vee(const Matrix3 V)
* Returns the vee map of a given 3x3 matrix. This is the inverse of hat map.
* @param V matrix which the vee map is needed to be operated on
* @return vee map of the input matrix
*/
Vector3 vee(const Matrix3 V);


/** \fn void saturate(Vector3 &x, const double x_min, const double x_max)
 * Saturate the elements of a given 3x1 vector between a minimum and a maximum
 * value.
 * @param x     vector which the elements needed to be saturated
 * @param x_min minimum value for each element
 * @param x_max maximum value for each element
 */
void saturate(Vector3 &x, const double x_min, const double x_max);


/** \fn deriv_unit_vector(const Vector3 &A, const Vector3 &A_dot, \
 * const Vector3 A_ddot, Vector3 &q, Vector3 &q_dot, Vector3 &q_ddot)
 * Outputs the time derivatives of a vector after normalizing it.
 * @param A Non-normal vector
 * @param A_dot Time derivative of A
 * @param A_ddot Time derivative of A_dot
 * @param 
 */
void deriv_unit_vector( \
    const Vector3 &A, const Vector3 &A_dot, const Vector3 &A_ddot, \
    Vector3 &q, Vector3 &q_dot, Vector3 &q_ddot
);


#endif
