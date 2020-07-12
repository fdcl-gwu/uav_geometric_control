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
%%
close all;
addpath('aux_functions');
addpath('test_functions');

%% Simulation parameters
t = 0:0.01:10;
N = length(t);

% Quadrotor
J1 = 0.02;
J2 = 0.02;
J3 = 0.04;
param.J = diag([J1, J2, J3]);

param.m = 2;

param.d = 0.169;
param.ctf = 0.0135;

% Fixed disturbance
param.x_delta = [0.5, 0.8, -1]';
param.R_delta = [0.2, 1.0, -0.1]';

% Other parameters
param.g = 9.81;

%% Controller gains
k.x = 10;
k.v = 8;
k.i = 10;
param.c1 = 1.5;
param.sigma = 10;

% Attitude
k.R = 1.5;
k.W = 0.35;
k.I = 10;
param.c2 = 2;

% Yaw
k.y = 0.8;
k.wy = 0.15;
k.yI = 2;
param.c3 = 2;

%% Initial conditions
x0 = [0, 0, 0]';
v0 = [0, 0, 0]';
R0 = expm(pi * hat([0, 0, 1]'));
W0 = [0, 0, 0]';
X0 = [x0; v0; W0; reshape(R0,9,1); zeros(6,1)];

%% Numerical integration
[t, X] = ode45(@(t, XR) eom(t, XR, k, param), t, X0, ...
    odeset('RelTol', 1e-6, 'AbsTol', 1e-6));

%% Post processing

% Create empty arrays to save data
[e, d, R, f, M] = generate_output_arrays(N);

% Unpack the outputs of ode45 function
x = X(:, 1:3)';
v = X(:, 4:6)';
W = X(:, 7:9)';
ei = X(:, 19:21)';
eI = X(:, 22:24)';

for i = 1:N
    R(:,:,i) = reshape(X(i,10:18), 3, 3);
    
    des = command(t(i));
    [f(i), M(:,i), ~, ~, err, calc] = position_control(X(i,:)', des, ...
        k, param);
    
    % Unpack errors
    e.x(:,i) = err.x;
    e.v(:,i) = err.v;
    e.R(:,i) = err.R;
    e.W(:,i) = err.W;
    e.y(i) = err.y;
    e.Wy(i) = err.Wy;
    
    % Unpack desired values
    d.x(:,i) = des.x;
    d.v(:,i) = des.v;
    d.b1(:,i) = des.b1;
    d.R(:,:,i) = calc.R;
end

% Plot data
linetype = 'k';
linewidth = 1;
xlabel_ = 'time (s)';

figure;
plot_3x1(t, e.R, '', xlabel_, 'e_R', linetype, linewidth)
set(gca, 'FontName', 'Times New Roman');

figure;
plot_3x1(t, e.x, '', xlabel_, 'e_x', linetype, linewidth)
set(gca, 'FontName', 'Times New Roman');

figure;
plot_3x1(t, e.v, '', xlabel_, 'e_v', linetype, linewidth)
set(gca, 'FontName', 'Times New Roman');

figure;
plot_3x1(t, eI .* [k.I, k.I, k.yI]', '', xlabel_, 'e', linetype, linewidth)
plot_3x1(t, param.R_delta .* ones(3, N), ...
    '', xlabel_, 'e_I', 'r', linewidth)
set(gca, 'FontName', 'Times New Roman');

figure;
plot_3x1(t, ei * k.i, '', xlabel_, 'e_i', linetype, linewidth)
plot_3x1(t, param.x_delta .* ones(3, N), ...
    '', xlabel_, 'e_i', 'r', linewidth)
set(gca, 'FontName', 'Times New Roman');

figure;
plot_3x1(t, x, '', xlabel_, 'x', linetype, linewidth)
plot_3x1(t, d.x, '', xlabel_, 'x', 'r', linewidth)
set(gca, 'FontName', 'Times New Roman');

figure;
plot3(x(1,:), x(2,:), x(3,:), 'k');
hold on;
plot3(d.x(1,:), d.x(2,:), d.x(3,:), 'r');
set(gca, 'YDir', 'reverse', 'ZDir', 'reverse');
axis equal;
xlabel('$x_1$', 'interpreter', 'latex');
ylabel('$x_2$', 'interpreter', 'latex');
zlabel('$x_3$', 'interpreter', 'latex');
set(gca, 'Box', 'on');
grid on;
set(gca, 'FontName', 'Times New Roman');