function [RD]=Desired_R_att_cont(t)
% Define the desired attitude and its derivetive for attitude controller
% here:
R_d=eye(3,3);
R_d_dot=zeros(3,3);
R_dd_dot=zeros(3,3);
RD=[R_d,R_d_dot,R_dd_dot];


