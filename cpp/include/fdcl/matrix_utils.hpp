/** \file misc_matrix_func.h
*  \brief Miscellaneous math functions
*
*  Miscellaneous math functions used in FDCL are defined here
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
 * velue.
 * @param x     vector which the elements needed to be saturated
 * @param x_min minimum value for each element
 * @param x_max maximum value for each element
 */
void saturate(Vector3 &x, const double x_min, const double x_max);


#endif
