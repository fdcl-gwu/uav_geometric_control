function [x_d,xd_dot,xd_2dot,xd_3dot,xd_4dot]=Desired_x(t)
% For position controller, the desired position and its derivative are
% defined here:
a=[0.002;0.002;0.0005];
x_d=a*t;
xd_dot=a;

xd_2dot=zeros(3,1);
xd_3dot=zeros(3,1);
xd_4dot=zeros(3,1);
