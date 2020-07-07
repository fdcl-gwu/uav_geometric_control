/** \file control.hpp
*  \brief Structures to handle integral errors
*/

#ifndef FDCL_INTEGRAL_UTILS_HPP
#define FDCL_INTEGRAL_UTILS_HPP

// user headers
#include "common_types.hpp"
#include "matrix_operations.hpp"


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