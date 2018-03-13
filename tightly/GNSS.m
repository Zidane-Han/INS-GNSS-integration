%% GNSS_SPV

load('Time');
%load ('GPSsvECEF.mat');
%load('Pos_ecef_real');   % [m; m; m]
%load('Vel_ecef_real');   % [m/s; m/s;m/s]
GPSsvECEF=0;
Pos_ecef=[0;0;0];
Vel_ecef_real=[0;0;0];
%--------------------------------------------------------------------------
%% GNSS卫星ECEF坐标系下位置速度解算
PRNVis = [5,6,7,9,10,12,14,18,30,31];
PRNUnVis = [1,3,8,13,16,17,19,20,22,23,25,26,27,28];

tm = 500;
t0 = Time(1);

GNSS1Pos(1:t0,1) = GPSsv.rx(tm:t0+tm-1,5);GNSS1Pos(1:t0,2) = GPSsv.ry(tm:t0+tm-1,5);GNSS1Pos(1:t0,3) = GPSsv.rz(tm:t0+tm-1,5);
GNSS1V(1:t0,1) = GPSsv.vx(tm:t0+tm-1,5);  GNSS1V(1:t0,2) = GPSsv.vy(tm:t0+tm-1,5);  GNSS1V(1:t0,3) = GPSsv.vz(tm:t0+tm-1,5);

GNSS2Pos(1:t0,1) = GPSsv.rx(tm:t0+tm-1,6);GNSS2Pos(1:t0,2) = GPSsv.ry(tm:t0+tm-1,6);GNSS2Pos(1:t0,3) = GPSsv.rz(tm:t0+tm-1,6);
GNSS2V(1:t0,1) = GPSsv.vx(tm:t0+tm-1,6);  GNSS2V(1:t0,2) = GPSsv.vy(tm:t0+tm-1,6);  GNSS2V(1:t0,3) = GPSsv.vz(tm:t0+tm-1,6);

GNSS3Pos(1:t0,1) = GPSsv.rx(tm:t0+tm-1,7);GNSS3Pos(1:t0,2) = GPSsv.ry(tm:t0+tm-1,7);GNSS3Pos(1:t0,3) = GPSsv.rz(tm:t0+tm-1,7);
GNSS3V(1:t0,1) = GPSsv.vx(tm:t0+tm-1,7);  GNSS3V(1:t0,2) = GPSsv.vy(tm:t0+tm-1,7);  GNSS3V(1:t0,3) = GPSsv.vz(tm:t0+tm-1,7);

GNSS4Pos(1:t0,1) = GPSsv.rx(tm:t0+tm-1,9);GNSS4Pos(1:t0,2) = GPSsv.ry(tm:t0+tm-1,9);GNSS4Pos(1:t0,3) = GPSsv.rz(tm:t0+tm-1,9);
GNSS4V(1:t0,1) = GPSsv.vx(tm:t0+tm-1,9);  GNSS4V(1:t0,2) = GPSsv.vy(tm:t0+tm-1,9);  GNSS4V(1:t0,3) = GPSsv.vz(tm:t0+tm-1,9);

%% 真实伪距/率、GNSS伪距/率计算
c = 3.0e+8;

BiasTu = c*1.0e-8; 
PseudoNoise_Value = 3;
MeasPseudoNoise_Value = 3;
deltaBiasTu = BiasTu*ones(1,t0);
PseudoNoise = PseudoNoise_Value*randn(1,t0); 
MeasPseudoNoise = MeasPseudoNoise_Value*randn(1,t0); 

BiasTru = c*1.0e-10;
PseudoRateNoise_Value = 0.03;
MeasPseudoRateNoise_Value = 0.03;
deltaBiasTru = BiasTru*ones(1,t0);
PseudoRateNoise = PseudoRateNoise_Value*randn(1,t0); 
MeasPseudoRateNoise = MeasPseudoRateNoise_Value*randn(1,t0); 

for i = 1:t0
    
    if i==1
       deltaTru(1,1) = deltaBiasTru(1,1) + PseudoRateNoise(1,1);
       deltaTu(1,1) = deltaBiasTu(1,1) + deltaTru(1,1) + PseudoNoise(1,1);
    else
       deltaTru(1,i) = (-1/1800+1)*deltaTru(1,i-1) + PseudoRateNoise(1,i);
       deltaTu(1,i) = deltaBiasTu(1,i) + deltaTru(1,i) + PseudoNoise(1,i);  
    end
    
% 真实伪距、伪距率计算    
    Pseudoreal(1,i) = sqrt((Pos_ecef_real(1,i)-GNSS1Pos(i,1))^2+(Pos_ecef_real(2,i)-GNSS1Pos(i,2))^2+(Pos_ecef_real(3,i)-GNSS1Pos(i,3))^2);
    Pseudoreal(2,i) = sqrt((Pos_ecef_real(1,i)-GNSS2Pos(i,1))^2+(Pos_ecef_real(2,i)-GNSS2Pos(i,2))^2+(Pos_ecef_real(3,i)-GNSS2Pos(i,3))^2);
    Pseudoreal(3,i) = sqrt((Pos_ecef_real(1,i)-GNSS3Pos(i,1))^2+(Pos_ecef_real(2,i)-GNSS3Pos(i,2))^2+(Pos_ecef_real(3,i)-GNSS3Pos(i,3))^2);
    Pseudoreal(4,i) = sqrt((Pos_ecef_real(1,i)-GNSS4Pos(i,1))^2+(Pos_ecef_real(2,i)-GNSS4Pos(i,2))^2+(Pos_ecef_real(3,i)-GNSS4Pos(i,3))^2);   
    
    PseudoRatereal(1,i) = ((Pos_ecef_real(1,i)-GNSS1Pos(i,1))*(Vel_ecef_real(1,i)-GNSS1V(i,1)) + (Pos_ecef_real(2,i)-GNSS1Pos(i,2))*(Vel_ecef_real(2,i)-GNSS1V(i,2)) + (Pos_ecef_real(3,i)-GNSS1Pos(i,3))*(Vel_ecef_real(3,i)-GNSS1V(i,3))) / Pseudoreal(1,i);
    PseudoRatereal(2,i) = ((Pos_ecef_real(1,i)-GNSS2Pos(i,1))*(Vel_ecef_real(1,i)-GNSS2V(i,1)) + (Pos_ecef_real(2,i)-GNSS2Pos(i,2))*(Vel_ecef_real(2,i)-GNSS2V(i,2)) + (Pos_ecef_real(3,i)-GNSS2Pos(i,3))*(Vel_ecef_real(3,i)-GNSS2V(i,3))) / Pseudoreal(2,i);
    PseudoRatereal(3,i) = ((Pos_ecef_real(1,i)-GNSS3Pos(i,1))*(Vel_ecef_real(1,i)-GNSS3V(i,1)) + (Pos_ecef_real(2,i)-GNSS3Pos(i,2))*(Vel_ecef_real(2,i)-GNSS3V(i,2)) + (Pos_ecef_real(3,i)-GNSS3Pos(i,3))*(Vel_ecef_real(3,i)-GNSS3V(i,3))) / Pseudoreal(3,i);
    PseudoRatereal(4,i) = ((Pos_ecef_real(1,i)-GNSS4Pos(i,1))*(Vel_ecef_real(1,i)-GNSS4V(i,1)) + (Pos_ecef_real(2,i)-GNSS4Pos(i,2))*(Vel_ecef_real(2,i)-GNSS4V(i,2)) + (Pos_ecef_real(3,i)-GNSS4Pos(i,3))*(Vel_ecef_real(3,i)-GNSS4V(i,3))) / Pseudoreal(4,i);
% GPS伪距、伪距率计算   
    Pseudo_gps(1,i) = Pseudoreal(1,i) - deltaTu(1,i) - MeasPseudoNoise(1,i);
    Pseudo_gps(2,i) = Pseudoreal(2,i) - deltaTu(1,i) - MeasPseudoNoise(1,i);
    Pseudo_gps(3,i) = Pseudoreal(3,i) - deltaTu(1,i) - MeasPseudoNoise(1,i);
    Pseudo_gps(4,i) = Pseudoreal(4,i) - deltaTu(1,i) - MeasPseudoNoise(1,i);
    
    PseudoRate_gps(1,i) = PseudoRatereal(1,i) - deltaTru(1,i) - MeasPseudoRateNoise(1,i);
    PseudoRate_gps(2,i) = PseudoRatereal(2,i) - deltaTru(1,i) - MeasPseudoRateNoise(1,i);
    PseudoRate_gps(3,i) = PseudoRatereal(3,i) - deltaTru(1,i) - MeasPseudoRateNoise(1,i);
    PseudoRate_gps(4,i) = PseudoRatereal(4,i) - deltaTru(1,i) - MeasPseudoRateNoise(1,i); 
end
%--------------------------------------------------------------------------
save('GNSS1Pos');save('GNSS2Pos');save('GNSS3Pos');save('GNSS4Pos');
save('GNSS1V');save('GNSS2V');save('GNSS3V');save('GNSS4V');
save('Pseudoreal');save('PseudoRatereal'); 
save('Pseudo_gps');save('BiasTu');save('PseudoNoise_Value');save('MeasPseudoNoise_Value');
save('PseudoRate_gps');save('BiasTru');save('PseudoRateNoise_Value');save('MeasPseudoRateNoise_Value');
