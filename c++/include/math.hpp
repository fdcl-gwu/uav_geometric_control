# define M_PI           3.14159265358979323846  /* pi */

#ifndef AUX_FUNCTIONS_H
#define AUX_FUNCTIONS_H
#include <eigen3/Eigen/Dense>
#include <iostream>

using namespace Eigen;

void vec_average(MatrixXd& M_in, Vector3d& v_in){
  MatrixXd temp(M_in.rows(), M_in.cols());
  temp.block(0,1,3,M_in.cols()-1) = M_in.block(0,0,3,M_in.cols()-1);
  temp.block(0,0,3,1) = v_in;
  M_in = temp;
  return;
}

Vector3d vee_eigen(const Matrix3d& xhat){
    return Vector3d{xhat(2,1), xhat(0,2), xhat(1,0)};
}

void err_sat(double min_sat, double max_sat, Vector3d& err){

    for(int i = 0; i < 3; i++){
        if  (err(i) < min_sat)
            err(i) = min_sat;
        else if(err(i) > max_sat)
            err(i) = max_sat;
    }
    return;
}

Matrix3d hat_eigen(Vector3d x){
    Matrix3d xhat;
    xhat(0,0) =   0.0; xhat(0,1) = -x(2); xhat(0,2) =  x(1);
    xhat(1,0) =  x(2); xhat(1,1) =   0.0; xhat(1,2) = -x(0);
    xhat(2,0) = -x(1); xhat(2,1) =  x(0); xhat(2,2) =   0.0;
    return xhat;
}

void eigen_skew (Eigen::Vector3d&  x, Eigen::Matrix3d& skewM)
{// Obtains 3x3 skew-symmetric matrix from 3x1 vector
    skewM(0,0) = 0;
    skewM(0,1) = -x(2);
    skewM(0,2) = x(1);
    skewM(1,0) = x(2);
    skewM(1,1) = 0;
    skewM(1,2) = -x(0);
    skewM(2,0) = -x(1);
    skewM(2,1) = x(0);
    skewM(2,2) = 0;
}

void eigen_invskew (Eigen::Matrix3d& skewM, Eigen::Vector3d& x)
{// Obtains 3x1 vector from its skew-symmetric 3x3 matrix
    x(0) = skewM(2,1);
    x(1) = skewM(0,2);
    x(2) = skewM(1,0);
}
#endif // AUX_H
