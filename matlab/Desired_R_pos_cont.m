function [RD]=Desired_R_pos_cont(m,g,kx,kv,R,R_dot,ex,ev,xd_2dot,xd_3dot,xd_4dot,f,f1)
% For position controller define the deisred attitude as follows:

% Define the disired direction of b_1 vector:
b_1_d=[1;0;0];
b_1_d_dot=zeros(3,1);
b_1_d_dot_dot=zeros(3,1);

% Compute the first and second derivative of e_v:
e_v_dot=g*[0;0;1]- f* R*[0;0;1]/m- xd_2dot;
f_dot=(kx* ev+kv* e_v_dot-m* xd_3dot)'* R(:,3)+f1'* R_dot(:,3);
e_v_dot_dot=(-f_dot* R(:,3)- f* R_dot(:,3)-m*xd_3dot)/m;

% Compute the direction of b_3 vector:
A=-f1;%-kx*ex-kv*ev-m*g*[0;0;1]+m*xd_2dot;
A_dot=-kx*ev-kv*e_v_dot+m*xd_3dot;
A_dot_dot=-kx*e_v_dot-kv*e_v_dot_dot+m*xd_4dot;
b_3_c=-A/norm(A);

% Compute the direction of b_2 vector:
C=-cross(b_3_c,b_1_d);
n_C=norm(C);
b_2_c=-C/n_C;

% Compute the direction of b_1 vector:
b_1_c=cross(b_2_c,b_3_c);

% Construct commanded R:
R_c=[b_1_c,b_2_c,b_3_c];

% Compute the derivative of b_3:
n_A=norm(A);
b_3_c_dot=-(A_dot/n_A)+(A*(A_dot'*A))/(n_A^3);

% Compute the derivative of b_2:
C_dot=-(cross(b_3_c_dot,b_1_d)+cross(b_3_c,b_1_d_dot));
b_2_c_dot=-(C_dot/n_C)+(C*(C_dot'*C))/(n_C^3);

% Compute the derivative of b_1:
b_1_c_dot=cross(b_2_c_dot,b_3_c)+cross(b_2_c,b_3_c_dot);

% Construct derivative of commanded R:
R_c_dot=[b_1_c_dot,b_2_c_dot,b_3_c_dot];

% Compute the second derivative of b_3:
E0=-A_dot_dot/n_A;
E1=2*(A'*A_dot)*A_dot/(n_A^3);
E2=(norm(A_dot)^2+A'*A_dot_dot)*A/(n_A^3);
E3=-3*(A'*A_dot)^2*A/(n_A^5);
b_3_c_dot_dot=E0+E1+E2+E3;

% Compute the second derivative of b_2:
C_dot_dot=-(cross(b_3_c_dot_dot,b_1_d)+2*cross(b_3_c_dot,b_1_d_dot)+cross(b_3_c,b_1_d_dot_dot));
E0=-C_dot_dot/n_C;
E1=2*(C'*C_dot)*C_dot/(n_C^3);
E2=(norm(C_dot)^2+C'*C_dot_dot)*C/(n_C^3);
E3=-3*(C'*C_dot)^2*C/(n_C^5);
b_2_c_dot_dot=E0+E1+E2+E3;

% Construct the second derivative of commanded R:
R_c_2dot=[cross(b_2_c_dot_dot,b_3_c)+2*cross(b_2_c_dot,b_3_c_dot)+cross(b_2_c,b_3_c_dot_dot),b_2_c_dot_dot,b_3_c_dot_dot];


RD=[R_c,R_c_dot,R_c_2dot];%,R_d_dot_dot];
