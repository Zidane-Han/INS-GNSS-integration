%% genAcl
function [fenn,fp,dlamta,delu] = genAcl(Acc_n_real,Pos_n2_real,Vel_n_real,RmRn,Wie)

fenn = Acc_n_real;
dlamta = Vel_n_real(1,:)/((RmRn(1,:)+Pos_n2_real(3,:))*cos(Pos_n2_real(1,:)));
delu = Vel_n_real(2,:)/(RmRn(2,:)+Pos_n2_real(3,:));
fp =[0 (dlamta+2*Wie)*sin(Pos_n2_real(1,:)) -(dlamta+2*Wie)*cos(Pos_n2_real(1,:));
          -(dlamta+2*Wie)*sin(Pos_n2_real(1,:)) 0 -delu;
         (dlamta+2*Wie)*cos(Pos_n2_real(1,:)) delu 0]*Vel_n_real; 