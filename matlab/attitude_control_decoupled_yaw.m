% 
% Copyright (c) 2020 Flight Dynamics and Control Lab
% 
% Permission is hereby granted, free of charge, to any person obtaining a
% copy of this software and associated documentation files (the 
% "Software"), to deal in the Software without restriction, including 
% without limitation the rights to use, copy, modify, merge, publish, 
% distribute, sublicense, and/or sell copies of the Software, and to permit
% persons to whom the Software is furnished to do so, subject to the 
% following conditions:
% 
% The above copyright notice and this permission notice shall be included
%  in all copies or substantial portions of the Software.
% 
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
% OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF 
% MERCHANTABILITY,FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
% IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
% CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
% TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
% SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

function [M, eI_dot, eb, ew, ey, ewy] = attitude_control_decoupled_yaw(...
    R, W, eI, ...  % states
    b3d, b3d_dot, b3d_ddot, b1c, wc3, wc3_dot, ...  % desired values
    k, param ...  % gains and parameters
)
% [M, eI_dot, eb, ew, ey, ewy] = attitude_control_decoupled_yaw(R, W, eI, 
% b3d, b3d_dot, b3d_ddot, b1c, wc3, wc3_dot, k, param)
%
% Decoupled-yaw attitude controller
% 
%   Caluclates control moments for a given set of desired attitude commands 
%   using a decoupled-yaw controller. This function uses the controller
%   defined in https://ieeexplore.ieee.org/document/8815189.
%   
%   Inputs:
%    R: (3x3 matrix) current attitude in SO(3)
%    W: (3x1 matrix) current angular velocity
%    eI: (3x1 matrix) attitude integral error
%    b3d: (3x1 matrix) desired direction of b3 axis
%    b3d_dot: (3x1 matrix) desired direction of b3 axis
%    b3d_ddot: (3x1 matrix) desired rotational rate of b3 axis
%    b1c: (3x1 matrix) desired direction of b1 axis
%    wc3: (3x1 matrix) desired yaw angular velocity
%    wc3_dot: (3x1 matrix) desired yaw angular acceleration
%    k: (struct) control gains
%    param: (struct) parameters such as m, g, J in a struct
%
%  Outputs:
%    M: (3x1 matrix) control moment required to reach desired conditions
%    eI_dot: (3x1 matrix) attitude integral change rate
%    eb: (3x1 matrix) roll/pitch angle error
%    ew: (3x1 matrix) roll/pitch angular velocity error
%    ey: (3x1 matrix) yaw angle error
%    ewy: (3x1 matrix) yaw angular velocity error

%% Unpack other parameters
J = param.J;
c2 = param.c2;
c3 = param.c3;

%% Body axes
e1 = [1, 0, 0]';
e2 = [0, 1 ,0]';
e3 = [0, 0, 1]';

b1 = R * e1;
b2 = R * e2;
b3 = R * e3;

%% Roll/pitch dynamics
kb = k.R;
kw = k.W;

w = W(1) * b1 + W(2) * b2;
b3_dot = hat(w) * b3;

wd = hat(b3d) * b3d_dot;
wd_dot = hat(b3d) * b3d_ddot;

eb = hat(b3d) * b3;
ew = w + hat(b3)^2 * wd;
tau = - kb * eb ...
    - kw * ew ...
    - J(1,1) * dot(b3, wd) * b3_dot ...
    - J(1,1) * hat(b3)^2 * wd_dot ...
    - k.I * eI(1) * b1 - k.I * eI(2) * b2;

tau1 = dot(b1, tau);
tau2 = dot(b2, tau);

M1 = tau1 + J(3,3) * W(3) * W(2);
M2 = tau2 - J(3,3) * W(3) * W(1);

%% Yaw dynamics
ey = -dot(b2, b1c);
ewy = W(3) - wc3;

M3 = - k.y * ey ...
    - k.wy * ewy ...
    - k.yI * eI(3) ...
    + J(3,3) * wc3_dot;

eI_dot = [b1' * (c2 * eb + ew);
    b2' * (c2 * eb + ew);
    c3 * ey + ewy];

M=[M1, M2, M3]';

end