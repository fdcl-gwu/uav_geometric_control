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

function [M, eI_dot, eR, eW] = attitude_control( ...
    R, W, eI, ...  % states
    Rd, Wd, Wd_dot, ...  % desired values
    k, param ...  % gains and parameters
)
% [M, eI_dot, eR, eW] = attitude_control(R, W, eI, Rd, Wd, Wddot, k, param)
%
% Attitude controller
% 
%   Caluclates control moments for a given set of desired attitude commands 
%   using a the controller defined in 
%   https://ieeexplore.ieee.org/abstract/document/5717652
%   
%   Inputs:
%    R: (3x3 matrix) current attitude in SO(3)
%    W: (3x1 matrix) current angular velocity
%    eI: (3x1 matrix) attitude integral error
%    Rd: (3x3 matrix) desired attitude in SO(3)
%    Wd: (3x1 matrix) desired body angular velocity
%    Wd_dot: (3x1 matrix) desired body angular acceleration
%    k: (struct) control gains
%    param: (struct) parameters such as m, g, J in a struct
%
%  Outputs:
%    M: (3x1 matrix) control moment required to reach desired conditions
%    eI_dot: (3x1 matrix) attitude integral change rate
%    eR: (3x1 matrix) attitude error
%    eW: (3x1 matrix) angular velocity error

eR = 1 / 2 * vee(Rd' * R - R' * Rd);
eW = W - R' * Rd * Wd;

kR = diag([k.R, k.R, k.y]);
kW = diag([k.W, k.W, k.wy]);

M = - kR * eR ...
    - kW * eW ...
    - k.I * eI ...
    + hat(R' * Rd * Wd) * param.J * R' * Rd * Wd ...
    + param.J * R' * Rd * Wd_dot;

eI_dot = eW + param.c2 * eR;

end