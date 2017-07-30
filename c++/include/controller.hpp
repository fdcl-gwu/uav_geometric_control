#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "math.hpp"
#include <eigen3/Eigen/Dense>
using namespace Eigen;

struct node
{
Matrix3d J;
Vector4d f_motor;
Matrix3d R, Rc, Rc_dot, Rc_2dot;
Matrix3d R_conv;
Vector3d W, v, x, eX, eV, M, eiR, eR, eiR_last, eW, Wc, Wc_dot;
Vector3d b1d_ned, eiX, eiX_last, cX, eiX_sat;
float m, g, del_t, kx, kv, kiX, kRr, kW, kR, kiR, f_total, eiR_sat, cR;
};
class controller
{
private:
    node _params;
public:
    inline void set_mass(float m){this->_params.m = m;}
    inline float get_mass(){return _params.m;}
    static void GeometricPositionController(node& node, Vector3d xd, Vector3d xd_dot, Vector3d xd_ddot,Vector3d Wd, Vector3d Wddot, Vector3d x_v, Vector3d v_v, Vector3d W_in, Matrix3d R_v);
    static void GeometricControl_SphericalJoint_3DOF(node& node, Vector3d Wd, Vector3d Wddot, Vector3d W, Matrix3d R);
};
#endif
