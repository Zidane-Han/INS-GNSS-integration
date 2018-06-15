function Q=C_Q(Cnb)
%%姿态阵转四元数
 q0=0.5*(1+Cnb(1,1)+Cnb(2,2)+Cnb(3,3))^0.5;
% %q1=sign(q0)*sign(Cbn0(3,2)-Cbn0(2,3))*0.5*(1+Cbn0(1,1)-Cbn0(2,2)-Cbn0(3,3))^0.5;
% %q2=sign(q0)*sign(Cbn0(1,3)-Cbn0(3,1))*0.5*(1-Cbn0(1,1)+Cbn0(2,2)-Cbn0(3,3))^0.5;
% %q3=sign(q0)*sign(Cbn0(2,1)-Cbn0(1,2))*0.5*(1-Cbn0(1,1)-Cbn0(2,2)+Cbn0(3,3))^0.5;
 q1=0.25*(Cnb(3,2)-Cnb(2,3))/q0;
 q2=0.25*(Cnb(1,3)-Cnb(3,1))/q0;
 q3=0.25*(Cnb(2,1)-Cnb(1,2))/q0;
 modQ=sqrt(q0^2+q1^2+q2^2+q3^2);
Q=[q0;q1;q2;q3]/modQ;
