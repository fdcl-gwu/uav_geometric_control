function [U_sum,f1]=cal_controller_Usum_conti(t,R,x,v,k_x,k_v,m,g,xd,xd_dot,xd_2dot,M_law,Kxi,eiX_saturated,kXv)
ex=x-xd;
ev=v-xd_dot;
Integ=[0;0;0];
if M_law==23 % Paper: Geometric Nonlinear PID Control of a Quadrotor UAV on SE(3)
    Integ=-Kxi*eiX_saturated;
    k_x=kXv;
end
f1=(k_x*ex+k_v*ev+m*g*[0;0;1]-m*xd_2dot+Integ);
U_sum=f1'*R(:,3);
end