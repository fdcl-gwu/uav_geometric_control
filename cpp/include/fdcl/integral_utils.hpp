/** \file integral_utils.hpp
*  \brief Structures to handle integral errors
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

#ifndef FDCL_INTEGRAL_UTILS_HPP
#define FDCL_INTEGRAL_UTILS_HPP

// user headers
#include "common_types.hpp"
#include "fdcl/matrix_utils.hpp"


// external headers
#include "Eigen/Dense"


namespace fdcl
{

/** \brief Integral for error for Vector3
 */
struct integral_error_vec3
{
    Vector3 error;
    Vector3 integrand;

    integral_error_vec3(void)
    {
        set_zero();
    }


    /** \fn void integrate(const Vector3 current_integrand, const double dt)
     * 
     * Integration of errors for a 3x1 state.
     * 
     * @param current_integrand the value of the integrand right now
     * @param dt time step
     */
    void integrate(const Vector3 current_integrand, const double dt)
    {
        error += (integrand + current_integrand) * dt / 2;
        integrand = current_integrand;
    }


    /** \fn void set_zero(void)
     * Set all the errors to zero.
     */
    void set_zero(void)
    {
        error.setZero();
        integrand.setZero();
    }
};  // end of integral_error3 struct


/** \brief Integral for error for a double
 */
struct integral_error
{
    double error;
    double integrand;

    integral_error(void)
    {
        set_zero();
    }


    /** \fn void integrate(const double current_integrand, const double dt)
     * 
     * Integration of errors for a single state.
     * 
     * @param current_integrand the value of the integrand right now
     * @param dt time step
     */
    void integrate(const double current_integrand, const double dt)
    {
        error += (integrand + current_integrand) * dt / 2;
        integrand = current_integrand;
    }


    /** \fn void set_zero(void)
     * Set all the errors to zero.
     */
    void set_zero(void)
    {
        error = 0.0;
        integrand = 0.0;
    }
}; // end of integral_error struct

}  // end of namespace fdcl

#endif