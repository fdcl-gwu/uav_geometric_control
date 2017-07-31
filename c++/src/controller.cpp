#include "controller.hpp"
using namespace Eigen;
using namespace std;

void Controller::GeometricPositionController(Vector3d xd, Vector3d xd_dot, Vector3d xd_2dot,Vector3d Wd, Vector3d Wd_dot, Vector3d x, Vector3d v, Vector3d W_in, Matrix3d R){
//   std::cout.precision(5);
  Vector3d xd_3dot, xd_4dot, b1d, b1d_dot, b1d_2dot;
  xd_3dot = xd_4dot = b1d_dot = b1d_2dot = Vector3d::Zero();
  b1d = _params.b1d_ned;
  Vector3d e3(0,0,1);
  // convention conversion
  Vector3d W = W_in;
  _params.R = R; _params.W = W; _params.v = v; _params.x = x;
  b1d_dot = _params.R_conv*b1d_dot;
  b1d_2dot = _params.R_conv*b1d_2dot;

  // Translational Error Functions
  Vector3d ex = x - xd;
  Vector3d ev = v - xd_dot;
  _params.eX = ex; _params.eV = ev;

  // Saturation function
  // _params.eiX = _params.eiX_last+_params.del_t*(ex+_params.cX*ev);
  // err_sat(-_params.eiX_sat, _params.eiX_sat, _params.eiX);
  _params.eiX_last = _params.eiX;

  // Force 'f' along negative b3-axis
  float kx = _params.kx;
  //float kxr = _params.kxr;
  float kv = _params.kv;
  float kiX = _params.kiX;
  float kR = _params.kR;
  float kRr = _params.kRr;
  float kW = _params.kW;
  float kiR = _params.kiR;
  float m = _params.m;
  float g = _params.g;

  //Matrix3d kxm = Vector3d(kx,kx,kx*kxr).asDiagonal();
  Vector3d A = -kx*ex-kv*ev-kiX*_params.eiX-m*g*e3+m*xd_2dot;
  Vector3d L = R*e3;
  Vector3d Ldot = R*hat_eigen(W)*e3;
  float f = -A.dot(R*e3);
  _params.f_total = f;

  // Intermediate Terms for Rotational Errors
  Vector3d ea = g*e3-f/m*L-xd_2dot;
  Vector3d Adot = -kx*ev-kv*ea+m*xd_3dot;// Lee Matlab: -ki*satdot(sigma,ei,ev+c1*ex);

  float fdot = -Adot.dot(L)-A.dot(Ldot);
  Vector3d eb = -fdot/m*L-f/m*Ldot-xd_3dot;
  Vector3d Ad_dot = -kx*ea-kv*eb+m*xd_4dot;// Lee Matlab: -ki*satdot(sigma,ei,ea+c1*ev);

  float nA = A.norm();
  Vector3d Ld = -A/nA;
  Vector3d Ld_dot = -Adot/nA+A*A.dot(Adot)/pow(nA,3);
  Vector3d Ld_2dot = -Ad_dot/nA+Adot/pow(nA,3)*(2*A.dot(Adot))
          +A/pow(nA,3)*(Adot.dot(Adot)+A.dot(Ad_dot))
          -3*A/pow(nA,5)*pow(A.dot(Adot),2);

  Vector3d Ld2 = -hat_eigen(b1d)*Ld;
  Vector3d Ld2_dot = -hat_eigen(b1d_dot)*Ld-hat_eigen(b1d)*Ld_dot;
  Vector3d Ld2_2dot = -hat_eigen(b1d_2dot)*Ld-2*hat_eigen(b1d_dot)*Ld_dot-hat_eigen(b1d)*Ld_2dot;

  float nLd2 = Ld2.norm();
  Vector3d Rd2 = Ld2/nLd2;
  Vector3d Rd2dot = Ld2_dot/nLd2-Ld2.dot(Ld_2dot)/pow(nLd2,3)*Ld2;
  Vector3d Rd2_2dot = Ld2_2dot/nLd2-Ld2.dot(Ld2_dot)/pow(nLd2,3)*Ld2_dot
          -Ld2_dot.dot(Ld2_dot)/pow(nLd2,3)*Ld2-Ld2.dot(Ld2_2dot)/pow(nLd2,3)*Ld2
          -Ld2.dot(Ld2_dot)/pow(nLd2,3)*Ld2_dot
          +3*pow(Ld2.dot(Ld2_dot),2)/pow(nLd2,5)*Ld2;

  Vector3d Rd1 = hat_eigen(Rd2)*Ld;
  Vector3d Rd1dot = hat_eigen(Rd2dot)*Ld+hat_eigen(Rd2)*Ld_dot;
  Vector3d Rd1_2dot = hat_eigen(Rd2_2dot)*Ld+2*hat_eigen(Rd2dot)*Ld_dot+hat_eigen(Rd2)*Ld_2dot;

  Matrix3d Rd, Rd_dot, Rd_2dot;
  Rd << Rd1, Rd2, Ld;
  Rd_dot << Rd1dot, Rd2dot, Ld_dot;
  Rd_2dot << Rd1_2dot, Rd2_2dot, Ld_2dot;
  _params.Rc = Rd;
  _params.Rc_dot = Rd_dot;
  _params.Rc_2dot = Rd_2dot;
  // Vector3d Wd, Wd_dot;
  Wd = vee_eigen(Rd.transpose()*Rd_dot);
  Wd_dot = vee_eigen(Rd.transpose()*Rd_2dot-hat_eigen(Wd)*hat_eigen(Wd));
  // TODO: _params.Wc and _params.Wc_dot
  _params.Wc = Wd; _params.Wc_dot = Wd_dot;
  // Attitude Error 'eR'
  _params.eR = vee_eigen(.5*(Rd.transpose()*R-R.transpose()*Rd));
  // Angular Velocity Error 'eW'
  _params.eW = W-R.transpose()*Rd*Wd;
  // Attitude Integral Term
  _params.eiR = _params.del_t*(_params.eR+_params.cR*_params.eW) + _params.eiR_last;
  err_sat(-_params.eiR_sat, _params.eiR_sat, _params.eiR);
  _params.eiR_last = _params.eiR;
  // 3D Moment
  Matrix3d kRm = Vector3d(kR,kR,kR*kRr).asDiagonal();
  _params.M = -kRm*_params.eR-kW*_params.eW-kiR*_params.eiR
    +hat_eigen(R.transpose()*Rd*Wd)*_params.J*R.transpose()*Rd*Wd
    +_params.J*R.transpose()*Rd*Wd_dot;

  Matrix<double, 4, 1> FM;
  FM[0] = f;
  FM[1] = _params.M[0];
  FM[2] = _params.M[1];
  FM[3] = _params.M[2];

  // _params.f_motor = _params.Ainv*FM;
}
//
// void controller::GeometricControl_SphericalJoint_3DOF(_params& _params, Vector3d Wd, Vector3d Wddot, Vector3d Win, Matrix3d Rin){
//
//   Matrix3d D = _params.R_conv;
//   Matrix3d R = D*Rin;// LI<-LBFF
//   Vector3d W = Win;// LBFF
//
//   Matrix3d Rd = MatrixXd::Identity(3,3);
//
//   Vector3d e3(0,0,1), b3(0,0,1), vee_3by1;
//   double l = 0;//.05;// length of rod connecting to the spherical joint
//   Vector3d r = -l * b3;
//   Vector3d F_g = _params.m * _params.g * R.transpose() * e3;
//   Vector3d M_g = r.cross(F_g);
//   //   Calculate eR (rotation matrix error)
//   Matrix3d inside_vee_3by3 = Rd.transpose() * R - R.transpose() * Rd;
//   eigen_invskew(inside_vee_3by3, vee_3by1);// 3x1
//   _params.eR = vee_3by1/2;
//   // Calculate eW (angular velocity error in body-fixed frame)
//   Vector3d eW = W - R.transpose() * Rd * Wd;
//   _params.eW = eW;
//   // Update integral term of control
//   // Attitude:
//   _params.eiR = _params.eiR_last + _params.del_t * _params.eR;
//   _params.eiR_last = _params.eiR;
//   // Calculate 3 DOFs of M (controlled moment in body-fixed frame)
//   // MATLAB: M = -kR*eR-kW*eW-kRi*eiR+cross(W,J*W)+J*(R'*Rd*Wddot-hat(W)*R'*Rd*Wd);
//   Matrix3d What;
//   eigen_skew(W, What);
//
//   Vector3d M = -_params.kR * _params.eR - _params.kW * eW - _params.kiR * _params.eiR + W.cross(_params.J*W) + _params.J*(R.transpose()*Rd*Wddot - What*R.transpose()*Rd*Wd) - M_g;
//   // To try different motor speeds, choose a force in the radial direction
//   double f = _params.m*_params.g;// N
//   _params.f_total = f;
//   // Convert forces & moments to f_i for i = 1:6 (forces of i-th prop)
//   VectorXd FM(4);
//   FM << f, M(0), M(1), M(2);
//   _params.M = M;
//   _params.f_motor = _params.Ainv * FM;
// }
