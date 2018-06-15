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

% 1)IMU��ʼ��������     
     Wb_BNf=mean(IMU(1:number_IMU_mean,5:7));                    % ��̬�ֶ�׼����õ������ݡ��Ӽƾ�ֵ�����������������ʱ��ƫ��;number_IMU_mean��LoadData�и�ֵ
     Fb_BNf=mean(IMU(1:number_IMU_mean,2:4))+[0 0 -1];
   
% 2)IMU��ʼ��������(�����С)         
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
     
% 3)IMU��ʼ��������(λ��P���ٶ�V����̬Z��Cnb��Qt��pwgs0_84)   

     %��ʼֵ�趨�����Կ��Ǹı��ʼֵ�鿴�������Ƿ��б�
     %��ʼλ�ã�GPS��ã����ȣ�γ�ȣ��߶ȣ� ��ʼ�ٶȣ�0 ��ʼ��̬����yaw-pitch-roll����ʼ��׼���
    % P0=[121.43695*d2r; 31.0237*d2r; 18.1];                      % initial position,P=position(longitude,latitude,height) set by GPS      
     V0=[0;0;0];                                                 % initial velocity,V=velocity    
    % Z0=[61;3;1]*d2r;                                            % initial attitude(unit:rad)��yaw-pitch-roll set by initial alignment
    Z0=[1.0648;0.0523;0.0175]*d2r;
    P0=[2.1195;0.5415;18.0998];
    P=P0;   V=V0;    Z=Z0;
     
     [Rm,Rn,g]= EarthConditionUpdate(P,Re,e,gideal);             %Rm��Rn�ֱ�Ϊ���������ڵ�����Ȧ��î��Ȧ�����ʰ뾶   �� �α�page341
     [wie,wen]= EarthRotationalVelocityUpdate(P,Rm,Rn,V,d2r);    %wie-������ת���ٶ� wen-λ������  �� �α�page300
     [Cnb,Qt] = CnbInitializationGet(Z);                         %��֪��̬Z�����������̬����Cnb����Ԫ��Qt�ĳ�ֵ�ɳ�ʼ��׼ȷ������̬�����
     [x0,y0,z0]=PInCC(P,Re,e);                                   %�ѿ�������
      pwgs0_84 =[x0;y0;z0];                                      % GPSλ������ pwgs0_84 - 84����ϵ
      oldV=V;

% 4)Kalman�˲����̱������� 
     KFVariableSet;                                              %�������������˲����� - �迴
     m=1;
     
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Part II:SINS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
tic
     for n=1:count                                                %count-����Ƶ��  30804
         Wbn=Wb(:,n);                                           %WbnȡWb�ĵ�count��
         Fbn=Fb(:,n);                                           %FbnȡFb�ĵ�count��
         %���ݲ���
         [wb,fb]= DataCompensation(Wbn,Wb_BNf,Fbn,Fb_BNf,Sf,Matrix_Wb,Matrix_Fb,d2r);   %Wb_BNf��Fb_BNf - ��̬�ֶ�׼����õ������ݡ��Ӽƾ�ֵ??
       
% 1)��̬����
       % wnb=wb-inv(Cnb)*(wie+wen);                               %�� �α�page300/9.2.50 
        wnb=(wb-[X_k(10)+X_k(13);X_k(11)+X_k(14);X_k(12)+X_k(15)])-inv(Cnb)*(wie+wen);                               %�� �α�page300/9.2.50 
        Wnb(:,n)=wnb;   
        if (n==1)   
            oldMqt=Mqt;
        end       
        Mqt=MqtGet(wnb);                                         %�� �α�page300/9.2.48 - M'    wnb - ������������ٶ�
        %****�Ľ�����-������***************             GPS/INS�α� page103
        Qt=QtUpdate(oldMqt,Mqt,T_IMU,Qt);                        %�����������������Ԫ��
        Cnb=CnbUpdate(Qt);                                       %�� �α�page 295/9.2.34 �ø��µ���Ԫ��������̬����
        Z=AttitudeUpdate(Cnb);                                   %�� �α�page 297/9.2.41 ��̬�Ǹ���          
        Zattitude(:,n)=Z;
        oldMqt=Mqt;        

%2) �ٶȸ���
        fn=Cnb*(fb-[X_k(16);X_k(17);X_k(18)])*g;  %???
        Fn(:,n)=fn;
        
        if (n==1)
            oldfn=fn;
        end

        cpfn= 0.5*(oldfn+fn);                                   %���
        cpv = VelocityCompensation(wie,wen);          
        
        %****�Ľ�����-������***************             GPS/INS�α� page103
        V= VelocityUpdate(oldfn,cpfn,cpv,fn,gn,g,T_IMU,V); 
        Velocity(:,n)=V;                            
        oldfn=fn; 

%3�� λ�ø���  
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
        
        if (mod(n,N_step)==0)                                                                % modȡn/N_step��
           % n 
           %************************KF*************************************
           %����״̬ϵͳ����
           %dx = F*x + G*w
           %z = H*x + v
           %��ɢ״̬ϵͳ����
           %x(k+1) = A*x(k) + B*w(k)
           %z(k+1) = C*x(k+1) + v(k+1)
           %���̼�GPS/INS�α� page260/ͼ7.2.2
           %�����ʼֵ -> ������I ->����F������ -> �������������� -> ״̬��������Э����Ԥ�� -> �������� -> ���
           
           Z_measure(1,1)=Z(3)-Z0(3);                                    % �����ߵ�/������� - �����ߵ�ϵͳ�͵�����������ĺ���ǵĲ�ֵ
           Z_measure(2,1)=P(3)-P0(3);                                    % P=[longtitude;latitude;h];   % �����ߵ�/��ȼ� - �����ߵ�ϵͳ��GPS��õĸ߶ȵĲ�ֵ
           Z_measure(3,1)=P(1)-P0(1);                                          % �����ߵ�/�˶�ģ�����λ�� - �����ߵ�ϵͳ���˶�ģ�����γ�ȵĲ�ֵ
           Z_measure(4,1)=P(2)-P0(2);                                      % �����ߵ�/�˶�ģ�����λ�� - �����ߵ�ϵͳ���˶�ģ��������ȵĲ�ֵ
           Z_measure(5,1)=V(1)-V0(1);
           Z_measure(6,1)=V(2)-V0(2);
           Z_measure(7,1)=V(3)-V0(3);
           
           Z_Measure(:,n/N_step)=Z_measure;                                                  % ���ⷽ�� Z_Measure=zeros(2,count_kf)
           
          
           [Ft,Gt,Ht]= KalmanMatrix(P,V,Rm,Rn,wie,fn,Cnb,T_Correlation);                %���״̬����ϵ��
           [Fdk,Gdk,Hdk,VQk,VRk,line]= KalmanMatrixDiscrete(Ft,Gt,Ht,VQ,VR,T_IMU);      %����״̬������ɢ�� - [״̬ת����,Gdk,Hdk,ϵͳ������Э������Qw,VRk,line]
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
