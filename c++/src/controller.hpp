#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "math.hpp"
#include <eigen3/Eigen/Dense>
using namespace Eigen;

struct node
{
Matrix4d J;
Vector4d f_motor;
Matrix3d R;
Matrix3d R_conv;
Vector3d W, v, x, eX, eV, M, eiR, kiR, eR, eiR_last, kR, eW, Wc, Wc_dot, cR;
Vector3d b1d_ned, eiX, eiX_last, cX, eiX_sat;
float m, g, del_t, kx, kv, kiX, kRr, kW;
};
class controller
{
public:
  static void GeometricPositionController(node& node, Vector3d xd, Vector3d xd_dot, Vector3d xd_ddot,Vector3d Wd, Vector3d Wddot, Vector3d x_v, Vector3d v_v, Vector3d W_in, Matrix3d R_v);
  static void GeometricControl_SphericalJoint_3DOF(node& node, Vector3d Wd, Vector3d Wddot, Vector3d W, Matrix3d R);
};
#endif
