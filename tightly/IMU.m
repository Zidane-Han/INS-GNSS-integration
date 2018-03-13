%% IMU元件的测量误差模型
%% 含Bias + Noise


load('Time');
load('Pos_n1_real');   % [m; m;m]
load('Pos_n2_real');   % [Lat(rad); Lon(rad);Hei(m)]
load('Vel_n_real');    % [m/s; m/s;m/s]
load('Acc_n_real');    % [m/s^2; m/s^2;m/s^2]

load('Vel_b_real');     % [m/s;m/s;m/s]
load('Acc_b_real');     % [m/s^2;m/s^2;m/s^2]

load('Attitude_real');   % [Pitch(rad);Roll(rad);Yaw(rad)]
load('RmRn');            % [Rm(m);Rn(m)]
load('Cnb_real');
load('Cne_real');

load('Pos_ecef_real');   % [m; m;m]
load('Vel_ecef_real');   % [m/s;m/s;m/s]
%--------------------------------------------------------------------------
d2r = pi/180;
Wie = 15/3600*d2r;
g = 9.8;
tDeltaAsqrt = 1;
t = 1;

BiasGyro = 1/3600*d2r;          % 战术级:1deg/h + 1e-3*g；导航级:0.01deg/h + 25*1e-6*g；战略级:0.001deg/h + 1e-6*g; 
WhiteGyro = 1/3600*d2r;
WhiteGyro1 = 1/3600*d2r;
% errBiasGyro = (BiasGyro*tDeltaAsqrt)*(2*rand(3,t0)-ones(3,t0));
errBiasGyro = (BiasGyro*tDeltaAsqrt)*ones(3,t0);
errWhite1Gyro = (WhiteGyro1*tDeltaAsqrt)*randn(3,t0);    %random error: 0.5deg/hr (1 sigma)
errWhiteGyro = (WhiteGyro*tDeltaAsqrt)*randn(3,t0);
kg = 0.0001;
KG = kg*(2*rand(3,t0)-ones(3,t0));

BiasAccel = 1*1e-3*g;   
WhiteAccel = 1*1e-3*g;
WhiteAccel1 = 1*1e-3*g;
% errBiasAccel = (BiasAccel*tDeltaAsqrt)*(2*rand(3,t0)-ones(3,t0));
errBiasAccel = (BiasAccel*tDeltaAsqrt)*ones(3,t0);
errWhite1Accel = (WhiteAccel1*tDeltaAsqrt)*randn(3,t0);   %random error: 1e-3g (1 sigma)
errWhiteAccel = (WhiteAccel*tDeltaAsqrt)*randn(3,t0); 
ka = 0.0001;
KA = ka*(2*rand(3,t0)-ones(3,t0));
fg = [0;0;-g];  

for i=1:tDeltaAsqrt:Time(1)
    
    if i==1
       deltaWnbb(:,1) = [0;0;0];
       errDrift1Gyro(:,1) = errWhite1Gyro(:,1); 
       errDrift1Accel(:,1) = errWhite1Accel(:,1); 
    else
    deltaWnbb(:,i) = (Attitude_real(:,i)-Attitude_real(:,i-1))/t;        
    errDrift1Gyro(:,i) = (-tDeltaAsqrt/1800+1)*errDrift1Gyro(:,i-1) + errWhite1Gyro(:,i);
    errDrift1Accel(:,i) = (-tDeltaAsqrt/1800+1)*errDrift1Accel(:,i-1) + errWhite1Accel(:,i);;
    end

    errDriftGyro(:,i) = errBiasGyro(:,i) + errWhiteGyro(:,i);             % b系下的陀螺角速度误差
    errDriftAccel(:,i) = errBiasAccel(:,i) + errWhiteAccel(:,i);          % b系下的加计加速度误差

    [wnbb(:,i),wien(:,i),wenn(:,i)] = genGyro(deltaWnbb(:,i),Attitude_real(:,i),Pos_n2_real(:,i),Vel_n_real(:,i),RmRn(:,i),Wie);
    errGM(:,i) = (diag(KG(:,i)))*(wnbb(:,i)+ Cnb_real(:,:,i)*(wien(:,i)+wenn(:,i)))+ errDriftGyro(:,i);
    wibb(:,i) = (eye(3)+diag(KG(:,i)))*(wnbb(:,i)+ Cnb_real(:,:,i)*(wien(:,i)+wenn(:,i)))+ errDriftGyro(:,i);% 陀螺仪仿真器的输出(含常值漂移和随机漂移)
    Wibb(:,i) = wibb(:,i);
    
    [fenn(:,i),fp(:,i),dlamta(:,1),delu(:,1)] = genAcl(Acc_n_real(:,i),Pos_n2_real(:,i),Vel_n_real(:,i),RmRn(:,i),Wie);
    errAM(:,i) = (diag(KA(:,i)))*Cnb_real(:,:,i)*(fenn(:,i)-fp(:,i)-fg) + errDriftAccel(:,i) ; 
    fb(:,i) = (eye(3)+diag(KA(:,i)))*Cnb_real(:,:,i)*(fenn(:,i)-fp(:,i)-fg) + errDriftAccel(:,i) ;            % 加速度计仿真器的输出(含常值漂移和随机漂移)
    Fb(:,i) = fb(:,i);
end
%--------------------------------------------------------------------------
save('Wibb');save('errGM');save('errDriftGyro');save('BiasGyro');save('WhiteGyro');save('kg');
save('Fb');save('errAM');save('errDriftAccel');save('BiasAccel');save('WhiteAccel');save('ka');
