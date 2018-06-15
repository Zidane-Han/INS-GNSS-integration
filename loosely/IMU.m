% 单独惯导解算
clear all
close all
clc
format long; 
%% 基本参数设定
global Re
global e
global gideal
global d2r

count=9000; %采样长度
Re   = 6378137;
e    = 1/298.257;
gideal= 9.78049;
d2r=pi/180;
Tt=0.02; %龙格库塔步长

%% INS设定
% 1)IMU角速度、比力输出值
     gn=[0;0;1];
     Wb_noise=wgn(3,1,10*log10(10e-12));
     Fb_noise=wgn(3,1,10*log10(10e-11));              
     wb=[1*d2r/3600;1*d2r/3600;1*d2r/3600]+[Wb_noise(1);Wb_noise(2);Wb_noise(3)];               
     fb=[0.001;0.001;-1.001]+[Fb_noise(1);Fb_noise(2);Fb_noise(3)];
     
% 2)初始变量定义      
     Zattitude=zeros(3,count);        Z=zeros(3,1);  
     Velocity=zeros(3,count);         V=zeros(3,1);                oldV=zeros(3,1); 
     Position=zeros(3,count);        % P=zeros(3,1);
     
     Pwgs_84=zeros(3,count);          pwgs_84=zeros(3,1);          pwgs0_84=zeros(3,1);
     Wenx=zeros(3,count);
     Wiex=zeros(3,count);
     GG=zeros(1,count);
     Rnx=zeros(1,count);
     Rmx=zeros(1,count);
     
     Wnb=zeros(3,count);              wnb=zeros(3,1); 
     Fn=zeros(3,count);               fn=zeros(3,1);               oldfn=zeros(3,1); 
     %wb=zeros(3,1);                   fb=zeros(3,1); 
     Cnb=zeros(3);                    cpv=zeros(3);      
     Qt=zeros(4,1);                   Mqt=zeros(4);                oldMqt=zeros(4);  %四元数        
     
% 3)IMU初始导航条件(位置P、速度V、姿态Z、Cnb、Qt、pwgs0_84)   
    %静止点 
    Z0=[0;0;0];    V0=[0;0;0];                                             
    P0=[degtorad(40);degtorad(115);10];   %地理坐标position(latitude,longtitude,altitude;纬经高) deg
    P=P0;   V=V0;    Z=Z0;
     %地球参数
    [Rm,Rn,g]= EarthConditionUpdate(P,Re,e,gideal);             %Rm、Rn分别为运载体所在点子午圈和卯酉圈的曲率半径   秦 page210
    [wie,wen]= EarthRotationalVelocityUpdate(P,Rm,Rn,V);   
    [Cnb,Qt] = CnbInitializationGet(Z);                         %存疑？？？根据姿态姿态矩阵Cnb、四元数Qt
    [x0,y0,z0]=PInCC(P,Rn,e);                                   %球面坐标系到直角坐标系
    pwgs0_84 =[x0;y0;z0];                                      % GPS位置坐标 pwgs0_84 - 84坐标系
    oldV=V;

% 4)Kalman滤波方程变量设置 
     m=1;
%% 
for n=1:count                                   
         %Wbn=Wb(:,n); Fbn=Fb(:,n);                                          
         %数据补偿
         %[wb,fb]= DataCompensation(Wbn,Wb_BNf,Fbn,Fb_BNf,Sf,Matrix_Wb,Matrix_Fb,d2r);   %Wb_BNf、Fb_BNf - 静态粗对准计算得到的陀螺、加计均值??
         
%% 惯导解算       
% 1)姿态更新
        wnb=wb-(Cnb)'*(wie+wen);                               %秦 课本page300/9.2.50 
       % wnb=(wb-[X_k(10)+X_k(13);X_k(11)+X_k(14);X_k(12)+X_k(15)])-inv(Cnb)*(wie+wen);                           
        Wnb(:,n)=wnb;   
        if (n==1)   
            oldMqt=Mqt;
        end       
        Mqt=MqtGet(wnb);                                         %秦 课本page300/9.2.49 - M' 
        %四阶龙格-库塔法解算
        Qt=QtUpdate(oldMqt,Mqt,Tt,Qt);                           %用龙格库塔法更新四元数
        Cnb=CnbUpdate(Qt);                                       %秦 课本page 295/9.2.34 用更新的四元数更新姿态矩阵
        Z=AttitudeUpdate(Cnb);                                   %秦 课本page 297/9.2.41 姿态角更新          
        Zattitude(:,n)=Z;
        oldMqt=Mqt;        
%2) 速度更新
        %fn=Cnb*(fb-[X_k(16);X_k(17);X_k(18)]);  %????
        fn=Cnb*fb*g;
        Fn(:,n)=fn;
        if (n==1)
            oldfn=fn;
        end
        cpfn= 0.5*(oldfn+fn);                               
        cpv = VelocityCompensation(wie,wen);       %这个有些问题
        %四阶龙格库塔法           
        V= VelocityUpdate(oldfn,cpfn,cpv,fn,gn,g,Tt,V); 
        Velocity(:,n)=V;                            
        oldfn=fn; 
%3） 位置更新  
        P= PositionUpdate(P,V,oldV,Tt,Rm,Rn);      
        [x0,y0,z0]= PInCC(P,Rn,e);     
        pwgs_84=[x0;y0;z0]-pwgs0_84;
        
        [Rm,Rn,g]= EarthConditionUpdate(P,Re,e,gideal);         
        [wie,wen]= EarthRotationalVelocityUpdate(P,Rm,Rn,V);   
        Position(:,n)=P;
        Pwgs_84(:,n)=pwgs_84; 
        oldV=V; 

end

t=0:Tt:Tt*(n-1);
figure(1),
subplot(3,1,1),plot(t,Zattitude(1,:)/d2r,'b','LineWidth',2),title('Attitude-Yaw (static)'),xlabel('Time(s)'),ylabel('Angular velocity(degree/s)'),grid on;
subplot(3,1,2),plot(t,Zattitude(2,:)/d2r,'g','LineWidth',2),title('Attitude-Pitch (static)'),xlabel('Time(s)'),ylabel('Angular velocity(degree/s)'),grid on;
subplot(3,1,3),plot(t,Zattitude(3,:)/d2r,'m','LineWidth',2),title('Attitude-Roll (static)'),xlabel('Time(s)'),ylabel('Angular velocity(degree/s)'),grid on;

figure(2),
subplot(3,1,1),plot(t,Velocity(1,:),'b','LineWidth',2),title('Ve (static)'),grid on;
subplot(3,1,2),plot(t,Velocity(2,:),'g','LineWidth',2),title('Vn (static)'),grid on;
subplot(3,1,3),plot(t,Velocity(3,:),'m','LineWidth',2),title('Vu (static)'),grid on;

figure(3),
subplot(3,1,1),plot(t,Re*(Position(1,:)-P0(1)),'b','LineWidth',2),title('Position-Latitude (static)'),grid on;
subplot(3,1,2),plot(t,Re*(Position(2,:)-P0(2)),'g','LineWidth',2),title('Position-Longtitude (static)'),grid on;
subplot(3,1,3),plot(t,Position(3,:)-P0(3),'m','LineWidth',2),title('Position-Altitude (static)'),grid on;
