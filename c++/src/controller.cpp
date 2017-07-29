#include <uav_geometric_controller/controller.hpp>
using namespace Eigen;
using namespace std;

void controller::GeometricPositionController(node& node, Vector3d xd, Vector3d xd_dot, Vector3d xd_2dot,Vector3d Wd, Vector3d Wd_dot, Vector3d x, Vector3d v, Vector3d W_in, Matrix3d R){
  std::cout.precision(5);
  Vector3d xd_3dot, xd_4dot, b1d, b1d_dot, b1d_2dot;
  xd_3dot = xd_4dot = b1d_dot = b1d_2dot = Vector3d::Zero();
  b1d = node.b1d_ned;
  Vector3d e3(0,0,1);
  // convention conversion
  Vector3d W = W_in;
  node.R = R; node.W = W; node.v = v; node.x = x;
  b1d_dot = node.R_conv*b1d_dot;
  b1d_2dot = node.R_conv*b1d_2dot;

  // Translational Error Functions
  Vector3d ex = x - xd;
  Vector3d ev = v - xd_dot;
  node.eX = ex; node.eV = ev;

  // Saturation function
  node.eiX = node.eiX_last+node.del_t*(ex+node.cX*ev);
  err_sat(-node.eiX_sat, node.eiX_sat, node.eiX);
  node.eiX_last = node.eiX;

  // Force 'f' along negative b3-axis
  float kx = node.kx;
  //float kxr = node.kxr;
  float kv = node.kv;
  float kiX = node.kiX;
  float kR = node.kR;
  float kRr = node.kRr;
  float kW = node.kW;
  float kiR = node.kiR;
  float m = node.m;
  float g = node.g;

  //Matrix3d kxm = Vector3d(kx,kx,kx*kxr).asDiagonal();
  Vector3d A = -kx*ex-kv*ev-kiX*node.eiX-m*g*e3+m*xd_2dot;
  Vector3d L = R*e3;
  Vector3d Ldot = R*hat_eigen(W)*e3;
  float f = -A.dot(R*e3);
  node.f_total = f;

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
  node.Rc = Rd;
  node.Rc_dot = Rd_dot;
  node.Rc_2dot = Rd_2dot;
  // Vector3d Wd, Wd_dot;
  vee_eigen(Rd.transpose()*Rd_dot, Wd);
  vee_eigen(Rd.transpose()*Rd_2dot-hat_eigen(Wd)*hat_eigen(Wd), Wd_dot);
  // TODO: node.Wc and node.Wc_dot
  node.Wc = Wd; node.Wc_dot = Wd_dot;
  // Attitude Error 'eR'
  vee_eigen(.5*(Rd.transpose()*R-R.transpose()*Rd), node.eR);
  // Angular Velocity Error 'eW'
  node.eW = W-R.transpose()*Rd*Wd;
  // Attitude Integral Term
  node.eiR = node.del_t*(node.eR+node.cR*node.eW) + node.eiR_last;
  err_sat(-node.eiR_sat, node.eiR_sat, node.eiR);
  node.eiR_last = node.eiR;
  // 3D Moment
  Matrix3d kRm = Vector3d(kR,kR,kR*kRr).asDiagonal();
  node.M = -kRm*node.eR-kW*node.eW-kiR*node.eiR+hat_eigen(R.transpose()*Rd*Wd)*node.J*R.transpose()*Rd*Wd+node.J*R.transpose()*Rd*Wd_dot;

  Matrix<double, 4, 1> FM;
  FM[0] = f;
  FM[1] = node.M[0];
  FM[2] = node.M[1];
  FM[3] = node.M[2];

  node.f_motor = node.Ainv*FM;
}

void controller::GeometricControl_SphericalJoint_3DOF(node& node, Vector3d Wd, Vector3d Wddot, Vector3d Win, Matrix3d Rin){

  Matrix3d D = node.R_conv;
  Matrix3d R = D*Rin;// LI<-LBFF
  Vector3d W = Win;// LBFF

  Matrix3d Rd = MatrixXd::Identity(3,3);

  Vector3d e3(0,0,1), b3(0,0,1), vee_3by1;
  double l = 0;//.05;// length of rod connecting to the spherical joint
  Vector3d r = -l * b3;
  Vector3d F_g = node.m * node.g * R.transpose() * e3;
  Vector3d M_g = r.cross(F_g);
  //   Calculate eR (rotation matrix error)
  Matrix3d inside_vee_3by3 = Rd.transpose() * R - R.transpose() * Rd;
  eigen_invskew(inside_vee_3by3, vee_3by1);// 3x1
  node.eR = vee_3by1/2;
  // Calculate eW (angular velocity error in body-fixed frame)
  Vector3d eW = W - R.transpose() * Rd * Wd;
  node.eW = eW;
  // Update integral term of control
  // Attitude:
  node.eiR = node.eiR_last + node.del_t * node.eR;
  node.eiR_last = node.eiR;
  // Calculate 3 DOFs of M (controlled moment in body-fixed frame)
  // MATLAB: M = -kR*eR-kW*eW-kRi*eiR+cross(W,J*W)+J*(R'*Rd*Wddot-hat(W)*R'*Rd*Wd);
  Matrix3d What;
  eigen_skew(W, What);

  Vector3d M = -node.kR * node.eR - node.kW * eW - node.kiR * node.eiR + W.cross(node.J*W) + node.J*(R.transpose()*Rd*Wddot - What*R.transpose()*Rd*Wd) - M_g;
  // To try different motor speeds, choose a force in the radial direction
  double f = node.m*node.g;// N
  node.f_total = f;
  // Convert forces & moments to f_i for i = 1:6 (forces of i-th prop)
  VectorXd FM(4);
  FM << f, M(0), M(1), M(2);
  node.M = M;
  node.f_motor = node.Ainv * FM;
}

void controller::gazebo_control(node& node){
  ros::ServiceClient client_FM = node.n_.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");
  gazebo_msgs::ApplyBodyWrench FMcmds_srv;

  Vector3d fvec_GB(0.0, 0.0, node.f_total), fvec_GI;

  fvec_GI = node.R_b*fvec_GB;
  Vector3d M_out = node.R_b*node.R_conv*node.M;

  FMcmds_srv.request.body_name = "quadrotor::base_link";
  FMcmds_srv.request.reference_frame = "world";
  FMcmds_srv.request.reference_point.x = 0.0;
  FMcmds_srv.request.reference_point.y = 0.0;
  FMcmds_srv.request.reference_point.z = 0.0;
  FMcmds_srv.request.start_time = ros::Time(0.0);
  FMcmds_srv.request.duration = ros::Duration(node.del_t);// apply continuously until new command

  double on_off = node.MOTOR_ON ? 1 : 0;
  FMcmds_srv.request.wrench.force.x = on_off * fvec_GI(0);
  FMcmds_srv.request.wrench.force.y = on_off * fvec_GI(1);
  FMcmds_srv.request.wrench.force.z = on_off * fvec_GI(2);

  FMcmds_srv.request.wrench.torque.x = on_off * M_out(0);
  FMcmds_srv.request.wrench.torque.y = on_off * M_out(1);
  FMcmds_srv.request.wrench.torque.z = on_off * M_out(2);

  client_FM.call(FMcmds_srv);
  // if(!FMcmds_srv.response.success)
  //     cout << "Fail! Response message:\n" << FMcmds_srv.response.status_message << endl;
}
