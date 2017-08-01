function [M,omega_d,omega_d_dot,e_R,e_omega]=cal_controller_M_conti(t,R,omega,k_R,k_omega,I,RD,M_law,KRi,eiR_saturated,kRv)

R_d=RD(1:3,1:3);
R_d_dot=RD(1:3,4:6);
R_dd_dot=RD(1:3,7:9);

omega_d=veemap(R_d'*R_d_dot);
omega_d_dot=veemap(R_d'*R_dd_dot-hatmap(omega_d)^2);

e_R=0.5*veemap(R_d'*R-R'*R_d);
e_omega=omega-R'*R_d*omega_d;

if M_law==1
    M=-k_R*e_R-k_omega*e_omega+cross(omega,I*omega)...
        -I*(hatmap(omega)*R'*R_d*omega_d-R'*R_d*omega_d_dot);
elseif M_law==2
    Rtdo=R'*R_d*omega_d;
    Rtdod=R'*R_d*omega_d_dot;
    M=-k_R*e_R-k_omega*e_omega+hatmap(Rtdo)*I*Rtdo+I*Rtdod;
elseif M_law==23
    Rtdo=R'*R_d*omega_d;
    Rtdod=R'*R_d*omega_d_dot;
    M=-kRv*e_R-k_omega*e_omega+cross(Rtdo,I*Rtdo)+I*Rtdod-KRi*eiR_saturated;
end
end