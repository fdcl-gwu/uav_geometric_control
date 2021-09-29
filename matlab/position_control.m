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

function [f, M, ei_dot, eI_dot, error, calculated] ...
    = position_control(X, desired, k, param)
% [f, M, ei_dot, eI_dot, error, calculated] = position_control(X, desired, 
% k, param)
%
% Position controller that uses decoupled-yaw controller as the attitude
% controller
% 
%   Caluclates the force and moments required for a UAV to reach a given 
%   set of desired position commands using a decoupled-yaw controller
%   defined in https://ieeexplore.ieee.org/document/8815189.
%   
%   Inputs:
%    X: (24x1 matrix) states of the system (x, v, R, W, ei, eI)
%    desired: (struct) desired states
%    k: (struct) control gains
%    param: (struct) parameters such as m, g, J in a struct
%
%  Outputs:
%    f: (scalar) required motor force
%    M: (3x1 matrix) control moment required to reach desired conditions
%    ei_dot: (3x1 matrix) position integral change rate
%    eI_dot: (3x1 matrix) attitude integral change rate
%    error: (struct) errors for attitude and position control (for data
%    logging)
%    calculated: (struct) calculated desired commands (for data logging)

% Use this flag to enable or disable the decoupled-yaw attitude controller.
use_decoupled = true;

% Unpack states
[x, v, R, W, ei, eI] = split_to_states(X);

sigma = param.sigma;
c1 = param.c1;
m = param.m;
g = param.g;
e3 = [0, 0, 1]';

error.x = x - desired.x;
error.v = v - desired.v;
A = - k.x * error.x ...
    - k.v * error.v ...
    - m * g * e3 ...
    + m * desired.x_2dot ...
    - k.i * sat(sigma, ei);

ei_dot = error.v + c1 * error.x;
b3 = R * e3;
f = -dot(A, b3);
ea = g * e3 ...
    - f / m * b3 ...
    - desired.x_2dot ...
    + param.x_delta / m;
A_dot = - k.x * error.v ...
    - k.v * ea ...
    + m * desired.x_3dot ...
    - k.i * satdot(sigma, ei, ei_dot);

ei_ddot = ea + c1 * error.v;
b3_dot = R * hat(W) * e3;
f_dot = -dot(A_dot, b3) - dot(A, b3_dot);
eb = - f_dot / m * b3 - f / m * b3_dot - desired.x_3dot;
A_ddot = - k.x * ea ...
    - k.v * eb ...
    + m * desired.x_4dot ...
    - k.i * satdot(sigma, ei, ei_ddot);

[b3c, b3c_dot, b3c_ddot] = deriv_unit_vector(-A, -A_dot, -A_ddot);

A2 = -hat(desired.b1) * b3c;
A2_dot = -hat(desired.b1_dot) * b3c - hat(desired.b1) * b3c_dot;
A2_ddot = - hat(desired.b1_2dot) * b3c ...
    - 2 * hat(desired.b1_dot) * b3c_dot ...
    - hat(desired.b1) * b3c_ddot;

[b2c, b2c_dot, b2c_ddot] = deriv_unit_vector(A2, A2_dot, A2_ddot);

b1c = hat(b2c) * b3c;
b1c_dot = hat(b2c_dot) * b3c + hat(b2c) * b3c_dot;
b1c_ddot = hat(b2c_ddot) * b3c ...
    + 2 * hat(b2c_dot) * b3c_dot ...
    + hat(b2c) * b3c_ddot;

Rc = [b1c, b2c, b3c];
Rc_dot = [b1c_dot, b2c_dot, b3c_dot];
Rc_ddot = [b1c_ddot, b2c_ddot, b3c_ddot];

Wc = vee(Rc' * Rc_dot);
Wc_dot = vee(Rc' * Rc_ddot - hat(Wc)^2);

W3 = dot(R * e3, Rc * Wc);
W3_dot = dot(R * e3, Rc * Wc_dot) ...
    + dot(R * hat(W) * e3, Rc * Wc);

%% Run attitude controller
if use_decoupled
    [M, eI_dot, error.b, error.W, error.y, error.Wy] ...
        = attitude_control_decoupled_yaw( ...
        R, W, eI, ...
        b3c, b3c_dot, b3c_ddot, b1c, W3, W3_dot, ...
        k, param);
    
    % Only used for comparison between two controllers
    error.R = 1 / 2 * vee(Rc' * R - R' * Rc);
else
    [M, eI_dot, error.R, error.W] = attitude_control( ...
        R, W, eI, ...
        Rc, Wc, Wc_dot, ...
        k, param);
    error.y = 0;
    error.Wy = 0;
end 
%% Saving data
calculated.b3 = b3c;
calculated.b3_dot = b3c_dot;
calculated.b3_ddot = b3c_ddot;
calculated.b1 = b1c;
calculated.R = Rc;
calculated.W = Wc;
calculated.W_dot = Wc_dot;
calculated.W3 = dot(R * e3, Rc * Wc);
calculated.W3_dot = dot(R * e3, Rc * Wc_dot) ...
    + dot(R * hat(W) * e3, Rc * Wc);
end