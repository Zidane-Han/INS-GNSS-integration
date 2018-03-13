%% genGyro
function [wnbb,wien,wenn] = genGyro(deltaWnbb,Attitude_real,Pos_n2_real,Vel_n_real,RmRn,Wie)


wnbb = [ cos(Attitude_real(1,:))*sin(Attitude_real(2,:)) cos(Attitude_real(2,:)) 0;
        -sin(Attitude_real(1,:)) 0 1;
        -cos(Attitude_real(1,:))*cos(Attitude_real(2,:)) sin(Attitude_real(2,:)) 0]*[deltaWnbb(3,:);deltaWnbb(1,:);deltaWnbb(2,:)]; 
% 由欧拉角法求解姿态wnbb 

wien = [          0;
             Wie*cos(Pos_n2_real(1,:));
             Wie*sin(Pos_n2_real(1,:))];
wenn = [-Vel_n_real(2,:)/(RmRn(1,:) + Pos_n2_real(3,:));
         Vel_n_real(1,:)/(RmRn(2,:) + Pos_n2_real(3,:));
         Vel_n_real(1,:)/(RmRn(2,:) + Pos_n2_real(3,:)) * tan(Pos_n2_real(1,:))];  % rad
