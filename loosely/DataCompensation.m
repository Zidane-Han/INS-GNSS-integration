% Datacompensation
%      gideal              	重力标准值 =9.78049;
%      gn                   地理坐标系（E-N-U）重力加速度分量方向=[0;0;-1];  
%      Matrix_Wb            E-N-U -> N-E-D转换矩阵=[0 -1 0;-1 0 0;0 0 -1];
%      Matrix_Fb            E-N-U -> N-E-D转换矩阵，question？=[0 -1 0;-1 0 0;0 0 -1]; 
%      Sf                   Scale Factor=[0.003,0.003,0.003,0.002,0.002,0.002]
%      Wb                   (3*SampleCounts) the angle velocity of three axis's gyros(unit:deg/s)
%      Fb                   (3*SampleCounts) the accelerated velocity of three axis's accelerometers(unit:gm/s^2)
%      number_IMU_mean      1000;   
%      Wb_BNf、 Fb_BNf                静态粗对准计算得到的陀螺、加计均值，用作后面捷联解算时的偏差，前1000个数据的平均值，      
%      wb                   陀螺/加计数据预处理，去除均值并校正...
%      fb                   ...wb:机体坐标系,fb：机体坐标系，前进方向为y轴，右手方向为x轴，向上为正

function [wb,fb]= DataCompensation(Wbn,Wb_BNf,Fbn,Fb_BNf,Sf,Matrix_Wb,Matrix_Fb,d2r)

         for i=1:3
             wb(i,1)=(Wbn(i)-Wb_BNf(i))/(1+Sf(i));   %Sf 刻度系数
             fb(i,1)=(Fbn(i)-Fb_BNf(i))/(1+Sf(i+3));    
         end
         wb=Matrix_Wb*wb*d2r;
         fb=Matrix_Fb*fb; 