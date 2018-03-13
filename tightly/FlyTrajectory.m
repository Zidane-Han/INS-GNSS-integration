%% FlyTrajectory 

load('Time');
load('AngleDesign');          % rad
load('VelocityDesign');       % m/s
load('initPos');
%--------------------------------------------------------------------------
%% 理想飞行轨迹P/V/A的计算
% 载体坐标系RFU
% 导航坐标系ENU，弹体到导航坐标系的变换矩阵Cbn
d2r = pi/180;
t=1;
Re=6378137;f=1/298.257223563;e=sqrt(0.00669437999013);  

for i=1:1:t0                                                    % n:ENU
       Cnb_real(:,:,i) = genCnbAngleDesign(AngleDesign(:,i));        
       Vel_n_real(:,i) = genVn(VelocityDesign(:,i),AngleDesign(:,i));        
    if i==1                                                       
       Acc_n_real(:,1)=[0;0;0];        
       Pos_n1_real(:,1)=[0;0;0];    %[m;m;m]
       Pos_n2_real(:,1) = initPos;  %[rad;rad;m]
       RmRn(:,1) = genRmRn(Re,f,e,Pos_n2_real(1,:));       
    else
       Acc_n_real(:,i) = (Vel_n_real(:,i)-Vel_n_real(:,i-1))/t;      
       Pos_n1_real(:,i) = Pos_n1_real(:,i-1) + Vel_n_real(:,i)*t;
       Pos_n2_real(1,i) = Pos_n2_real(1,i-1) + Vel_n_real(2,i-1)/(RmRn(1,i-1)+Pos_n2_real(3,i-1))*t;
       Pos_n2_real(2,i) = Pos_n2_real(2,i-1) + Vel_n_real(1,i-1)/((RmRn(2,i-1)+Pos_n2_real(3,i-1))*cos(Pos_n2_real(1,i-1)))*t;
       Pos_n2_real(3,i) = Pos_n2_real(3,i-1) + Vel_n_real(3,i-1)*t;  
       RmRn(:,i) = genRmRn(Re,f,e,Pos_n2_real(:,i)); 
    end   
end

for i=1:1:t0                                                      % b:RFU    
    Attitude_real(:,i)= [AngleDesign(1,i);AngleDesign(2,i);AngleDesign(3,i)];
    Acc_b_real(:,i) = Cnb_real(:,:,i)*Acc_n_real(:,i);   
    Vel_b_real(:,i) = Cnb_real(:,:,i)*Vel_n_real(:,i);    
           
    Cne_real(:,:,i) = genCne(Pos_n2_real(:,i));                   % e:ECEF
    Vel_ecef_real(:,i) = Cne_real(:,:,i)*Vel_n_real(:,i);    
    Pos_ecef_real(:,i) = genPecef(RmRn(:,i),e,Pos_n2_real(:,i));     
end
%--------------------------------------------------------------------------
%% 保存弹道P/V/A数据
save('Pos_n1_real');    % [m;m;Hei(m)]
save('Pos_n2_real');    % [Lat(rad);Lon(rad);Hei(m)]

save('Vel_n_real');     % [m/s;m/s;m/s]
save('Acc_n_real');     % [m/s^2;m/s^2;m/s^2]

save('Vel_b_real');     % [m/s;m/s;m/s]
save('Acc_b_real');     % [m/s^2;m/s^2;m/s^2]

save('Attitude_real');   % [Pitch(rad);Roll(rad);Yaw(rad)]
save('RmRn');            % [Rm(m);Rn(m)]

save('Pos_ecef_real');   % [m;m;m]
save('Vel_ecef_real');   % [m/s;m/s;m/s]

save('Cnb_real');
save('Cne_real');
