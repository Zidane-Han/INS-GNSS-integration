% INS_DMC_PS_central
% Description: 
% INS_DMC_PS_central algorithm in simple way

%% -------------------------------------------------------------------------------------------------------------------------
clear all
close all
clc
format long;         

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Part I:Initialization %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     LoadData_Static;

% 1)IMU初始物理条件     
     Wb_BNf=mean(IMU(1:number_IMU_mean,5:7));                    % 静态粗对准计算得到的陀螺、加计均值，用作后面捷联解算时的偏差;number_IMU_mean在LoadData中赋值
     Fb_BNf=mean(IMU(1:number_IMU_mean,2:4))+[0 0 -1];
   
% 2)IMU初始变量定义(矩阵大小)         
     Position=zeros(3,count);         P=zeros(3,1);                
     Velocity=zeros(3,count);         V=zeros(3,1);                oldV=zeros(3,1);           
     Zattitude=zeros(3,count);        Z=zeros(3,1); 
     Pwgs_84=zeros(3,count);          pwgs_84=zeros(3,1);          pwgs0_84=zeros(3,1);
     Wenx=zeros(3,count);
     Wiex=zeros(3,count);
     GG=zeros(1,count);
     Rnx=zeros(1,count);
     Rmx=zeros(1,count);
     
     Wnb=zeros(3,count);              wnb=zeros(3,1); 
     Fn=zeros(3,count);               fn=zeros(3,1);               oldfn=zeros(3,1); 
     wb=zeros(3,1);                   fb=zeros(3,1); 
     Cnb=zeros(3);                    cpv=zeros(3);      
     Qt=zeros(4,1);                   Mqt=zeros(4);                oldMqt=zeros(4);      
    
     Z_Measure=zeros(7,count_kf);     Z_measure=zeros(7,1);     
     
% 3)IMU初始导航条件(位置P、速度V、姿态Z、Cnb、Qt、pwgs0_84)   

     %初始值设定，可以考虑改变初始值查看仿真结果是否有变
     %初始位置：GPS获得（经度，纬度，高度） 初始速度：0 初始姿态：（yaw-pitch-roll）初始对准获得
    % P0=[121.43695*d2r; 31.0237*d2r; 18.1];                      % initial position,P=position(longitude,latitude,height) set by GPS      
     V0=[0;0;0];                                                 % initial velocity,V=velocity    
    % Z0=[61;3;1]*d2r;                                            % initial attitude(unit:rad)：yaw-pitch-roll set by initial alignment
    Z0=[1.0648;0.0523;0.0175]*d2r;
    P0=[2.1195;0.5415;18.0998];
    P=P0;   V=V0;    Z=Z0;
     
     [Rm,Rn,g]= EarthConditionUpdate(P,Re,e,gideal);             %Rm、Rn分别为运载体所在点子午圈和卯酉圈的曲率半径   秦 课本page341
     [wie,wen]= EarthRotationalVelocityUpdate(P,Rm,Rn,V,d2r);    %wie-地球自转角速度 wen-位置速率  秦 课本page300
     [Cnb,Qt] = CnbInitializationGet(Z);                         %已知姿态Z，可以求得姿态矩阵Cnb、四元数Qt的初值由初始对准确定的姿态阵求得
     [x0,y0,z0]=PInCC(P,Re,e);                                   %笛卡儿座标
      pwgs0_84 =[x0;y0;z0];                                      % GPS位置坐标 pwgs0_84 - 84坐标系
      oldV=V;

% 4)Kalman滤波方程变量设置 
     KFVariableSet;                                              %！！！卡尔曼滤波设置 - 需看
     m=1;
     
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Part II:SINS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
tic
     for n=1:count                                                %count-采样频率  30804
         Wbn=Wb(:,n);                                           %Wbn取Wb的第count列
         Fbn=Fb(:,n);                                           %Fbn取Fb的第count列
         %数据补偿
         [wb,fb]= DataCompensation(Wbn,Wb_BNf,Fbn,Fb_BNf,Sf,Matrix_Wb,Matrix_Fb,d2r);   %Wb_BNf、Fb_BNf - 静态粗对准计算得到的陀螺、加计均值??
       
% 1)姿态更新
       % wnb=wb-inv(Cnb)*(wie+wen);                               %秦 课本page300/9.2.50 
        wnb=(wb-[X_k(10)+X_k(13);X_k(11)+X_k(14);X_k(12)+X_k(15)])-inv(Cnb)*(wie+wen);                               %秦 课本page300/9.2.50 
        Wnb(:,n)=wnb;   
        if (n==1)   
            oldMqt=Mqt;
        end       
        Mqt=MqtGet(wnb);                                         %秦 课本page300/9.2.48 - M'    wnb - 陀螺仪输出角速度
        %****四阶龙格-库塔法***************             GPS/INS课本 page103
        Qt=QtUpdate(oldMqt,Mqt,T_IMU,Qt);                        %用龙格库塔法更新四元数
        Cnb=CnbUpdate(Qt);                                       %秦 课本page 295/9.2.34 用更新的四元数更新姿态矩阵
        Z=AttitudeUpdate(Cnb);                                   %秦 课本page 297/9.2.41 姿态角更新          
        Zattitude(:,n)=Z;
        oldMqt=Mqt;        

%2) 速度更新
        fn=Cnb*(fb-[X_k(16);X_k(17);X_k(18)])*g;  %???
        Fn(:,n)=fn;
        
        if (n==1)
            oldfn=fn;
        end

        cpfn= 0.5*(oldfn+fn);                                   %差分
        cpv = VelocityCompensation(wie,wen);          
        
        %****四阶龙格-库塔法***************             GPS/INS课本 page103
        V= VelocityUpdate(oldfn,cpfn,cpv,fn,gn,g,T_IMU,V); 
        Velocity(:,n)=V;                            
        oldfn=fn; 

%3） 位置更新  
        P= PositionUpdate(P,V,oldV,T_IMU,Rm,Rn);      
        [x0,y0,z0]= PInCC(P,Re,e);     
        pwgs_84=[x0;y0;z0]-pwgs0_84;
        
        [Rm,Rn,g]= EarthConditionUpdate(P,Re,e,gideal);         
        [wie,wen]= EarthRotationalVelocityUpdate(P,Rm,Rn,V,d2r);   
        
        Position(:,n)=P;
        Pwgs_84(:,n)=pwgs_84; 
        oldV=V; 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Part III:Integrated Kalman filter %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        if (mod(n,N_step)==0)                                                                % mod取n/N_step余
           % n 
           %************************KF*************************************
           %连续状态系统方程
           %dx = F*x + G*w
           %z = H*x + v
           %离散状态系统方程
           %x(k+1) = A*x(k) + B*w(k)
           %z(k+1) = C*x(k+1) + v(k+1)
           %流程见GPS/INS课本 page260/图7.2.2
           %读入初始值 -> Φ阵置I ->计算F及Φ阵 -> 计算噪声方差阵 -> 状态向量及其协方差预报 -> 量测修正 -> 输出
           
           Z_measure(1,1)=Z(3)-Z0(3);                                    % 捷联惯导/电磁罗盘 - 捷联惯导系统和电子罗盘输出的航向角的差值
           Z_measure(2,1)=P(3)-P0(3);                                    % P=[longtitude;latitude;h];   % 捷联惯导/深度计 - 捷联惯导系统和GPS获得的高度的差值
           Z_measure(3,1)=P(1)-P0(1);                                          % 捷联惯导/运动模型输出位置 - 捷联惯导系统和运动模型输出纬度的差值
           Z_measure(4,1)=P(2)-P0(2);                                      % 捷联惯导/运动模型输出位置 - 捷联惯导系统和运动模型输出经度的差值
           Z_measure(5,1)=V(1)-V0(1);
           Z_measure(6,1)=V(2)-V0(2);
           Z_measure(7,1)=V(3)-V0(3);
           
           Z_Measure(:,n/N_step)=Z_measure;                                                  % 量测方程 Z_Measure=zeros(2,count_kf)
           
          
           [Ft,Gt,Ht]= KalmanMatrix(P,V,Rm,Rn,wie,fn,Cnb,T_Correlation);                %求解状态方程系数
           [Fdk,Gdk,Hdk,VQk,VRk,line]= KalmanMatrixDiscrete(Ft,Gt,Ht,VQ,VR,T_IMU);      %连续状态方程离散化 - [状态转移阵Φ,Gdk,Hdk,系统噪声的协方差阵Qw,VRk,line]
           [Pk_diag,X_k]= Kalmanfilter(Fdk,Hdk,P_k,VQk,VRk,line,Z_measure,X_k);         %
            Pk_Diag(:,number+1)=Pk_diag;
            Xk(:,number+1)=X_k;           
            number=number+1;           
            
            Cnn=[1,-X_k(3),X_k(2);X_k(3),1,-X_k(1);-X_k(2),X_k(1),1];  
            Z(1)=Z(1)-X_k(1);
            Z(2)=Z(2)-X_k(2);
            Z(3)=Z(3)-X_k(3);
            Zattitude(:,n)=Z;
            Zattitude_IDP(:,m)=Z; 
            
            V(1)=V(1)-X_k(4);
            V(2)=V(2)-X_k(5);
            V(3)=V(3)-X_k(6);
            Velocity(:,n)=V;
            Velocity_IDP(:,m)=V;
            
            P(1)=P(1)-X_k(7);
            P(2)=P(2)-X_k(8);
            P(3)=P(3)-X_k(9);
            Position(:,n)=P;
            Position_IDP(:,m)=P;
            [x0,y0,z0]= PInCC(P,Re,e);    
            pwgs_84=[x0;y0;z0]-pwgs0_84;   
            Pwgs_84(:,n)=pwgs_84; 
            Pwgs_84_IDP(:,m)=pwgs_84; 
            
            Wenx(:,m)=wen;
            Wiex(:,m)=wie;
            GG(:,m)=g;
            Rnx(:,m)=Rn;
            Rmx(:,m)=Rm;
              
            m=m+1;
        end
        
     end
toc

plot_IMU_Static_KF_Hour;
