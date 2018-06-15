%18*18�ߵ�GPS����ϳ���
clear all
close all
clc

%% ���������趨
global Re
global e
global gideal
global d2r

count=8000; %��������
Re   = 6378137;
e    = 1/298.257;
gideal= 9.78049;
Tg = 0.02;
N_step=10;
Tt=0.02; %�����������
d2r=pi/180;

%% INS�趨
% 1)IMU���ٶȡ��������ֵ
     gn=[0;0;1];
     Wb_noise=wgn(3,1,10*log10(10e-12));
     Fb_noise=wgn(3,1,10*log10(10e-11));
     wb=[1*d2r/3600;1*d2r/3600;1*d2r/3600]+[Wb_noise(1);Wb_noise(2);Wb_noise(3)];               
     fb=[0.001;0.001;-1.001]+[Fb_noise(1);Fb_noise(2);Fb_noise(3)];
   
% 2)��ʼ��������      
     Zattitude=zeros(3,count);        Z=zeros(3,1);  
     Velocity=zeros(3,count);         V=zeros(3,1);                oldV=zeros(3,1); 
     Position=zeros(3,count);        
     
     Pwgs_84=zeros(3,count);          pwgs_84=zeros(3,1);          pwgs0_84=zeros(3,1);
     Wenx=zeros(3,count);
     Wiex=zeros(3,count);
     GG=zeros(1,count);
     Rnx=zeros(1,count);
     Rmx=zeros(1,count);
     
     Wnb=zeros(3,count);              wnb=zeros(3,1); 
     Fn=zeros(3,count);               fn=zeros(3,1);               oldfn=zeros(3,1); 
     Cnb=zeros(3);                    cpv=zeros(3);      
     Qt=zeros(4,1);                   Mqt=zeros(4);                oldMqt=zeros(4);  %��Ԫ��    
    
     Z_Measure=zeros(6,count/N_step);     Z_measure=zeros(6,1);     
     
% 3)IMU��ʼ��������(λ��P���ٶ�V����̬Z��Cnb��Qt��pwgs0_84)   
    %��ֹ�� 
    Z0=[0;0;0]*pi/180;    V0=[0;0;0];                                             
    P0=[degtorad(40);degtorad(115);10];   %��������position(latitude,longtitude,altitude;γ����) deg
    P=P0;   V=V0;    Z=Z0;
     %�������
    [Rm,Rn,g]= EarthConditionUpdate(P,Re,e,gideal);             %Rm��Rn�ֱ�Ϊ���������ڵ�����Ȧ��î��Ȧ�����ʰ뾶   �� page210
    [wie,wen]= EarthRotationalVelocityUpdate(P,Rm,Rn,V);   
    [Cnb,Qt] = CnbInitializationGet(Z);                         %������̬��̬����Cnb����Ԫ��Qt
    [x0,y0,z0]=PInCC(P,Rn,e);                                   %��������ϵ��ֱ������ϵ
    pwgs0_84 =[x0;y0;z0];                                       % GPSλ������ pwgs0_84 - 84����ϵ
    oldV=V;

% 4)Kalman�˲����̱������� 
     KFVariableSet;                                              %VQ VR���ɣ�����
     m=1;
%% 
for n=1:count                                   
         %Wbn=Wb(:,n); Fbn=Fb(:,n);                                          
         %���ݲ���
         %[wb,fb]= DataCompensation(Wbn,Wb_BNf,Fbn,Fb_BNf,Sf,Matrix_Wb,Matrix_Fb,d2r);   %Wb_BNf��Fb_BNf - ��̬�ֶ�׼����õ������ݡ��Ӽƾ�ֵ??
         
%% �ߵ�����       
% 1)��̬����
%         wnb=(wb-[X_k(10);X_k(11);X_k(12)])-(Cnb)'*(wie+wen);                               %�� �α�page300/9.2.50 
        wnb=wb-(Cnb)'*(wie+wen);  
        Wnb(:,n)=wnb;   
        if (n==1)   
            oldMqt=Mqt;
        end       
        Mqt=MqtGet(wnb);                                         %�� �α�page300/9.2.49 - M' 
        %�Ľ�����-����������
        Qt=QtUpdate(oldMqt,Mqt,Tt,Qt);                           %�����������������Ԫ��
        Cnb=CnbUpdate(Qt);                                       %�� �α�page 295/9.2.34 �ø��µ���Ԫ��������̬����
        Z=AttitudeUpdate(Cnb);                                   %�� �α�page 297/9.2.41 ��̬�Ǹ���          
        Zattitude(:,n)=Z;
        oldMqt=Mqt;       
%2) �ٶȸ���
%         fn=Cnb*(fb-[X_k(13);X_k(14);X_k(15)])*g;  
        fn=Cnb*fb*g;
        Fn(:,n)=fn;
        if (n==1)
            oldfn=fn;
        end
        cpfn= 0.5*(oldfn+fn);                               
        cpv = VelocityCompensation(wie,wen);       %Debug
        %�Ľ����������           
        V= VelocityUpdate(oldfn,cpfn,cpv,fn,gn,g,Tt,V); 
        Velocity(:,n)=V;                            
        oldfn=fn; 
%3�� λ�ø���  
        P= PositionUpdate(P,V,oldV,Tt,Rm,Rn);      
        [x0,y0,z0]= PInCC(P,Rn,e);     
        pwgs_84=[x0;y0;z0]-pwgs0_84;
        
        [Rm,Rn,g]= EarthConditionUpdate(P,Re,e,gideal);         
        [wie,wen]= EarthRotationalVelocityUpdate(P,Rm,Rn,V);   
        Position(:,n)=P;
        Pwgs_84(:,n)=pwgs_84; 
        oldV=V; 
%% �������˲�
        if (mod(n,N_step)==0)                                                                        
           Z_measure(1,1)=P(1)-P0(1);                                    % P=[longtitude;latitude;h];   % �����ߵ�/��ȼ� - �����ߵ�ϵͳ��GPS��õĸ߶ȵĲ�ֵ
           Z_measure(2,1)=P(2)-P0(2);                                          % �����ߵ�/�˶�ģ�����λ�� - �����ߵ�ϵͳ���˶�ģ�����γ�ȵĲ�ֵ
           Z_measure(3,1)=P(3)-P0(3);                                      % �����ߵ�/�˶�ģ�����λ�� - �����ߵ�ϵͳ���˶�ģ��������ȵĲ�ֵ
           Z_measure(4,1)=V(1)-V0(1);
           Z_measure(5,1)=V(2)-V0(2);
           Z_measure(6,1)=V(3)-V0(3);
           Z_measure(1:3,1)=Z_measure(1:3,1)+VRp_noise;                                                               
           Z_measure(4:6,1)=Z_measure(4:6,1)+VRv_noise;
           
           Z_Measure(:,n/N_step)=Z_measure;
           
           [Ft,Gt,Ht]= KalmanMatrix(P,V,Rm,Rn,wie,fn,Cnb);              
           [Fdk,Gdk,Hdk,VQk,VRk,line]= KalmanMatrixDiscrete(Ft,Gt,Ht,VQ,VR,Tt,N_step);      %��ɢ��
            % �������˲���
            P_kk1=Fdk*P_k*(Fdk)'+VQk;                                             
            K_k=P_kk1*(Hdk)'/(Hdk*P_kk1*(Hdk)'+VRk);                            
            P_k=(eye(line)-K_k*Hdk)*P_kk1;  
            %P_k=(eye(line)-K_k*Hdk)*P_kk1*(eye(line)-K_k*Hdk)'+K_k*VRk*(K_k)'; 
            X_k=Fdk*X_k+K_k*(Z_measure-Hdk*(Fdk*X_k));                                                 
           
            Pk_diag=sqrt(diag(P_k));   
            Pk_Diag(:,number+1)=Pk_diag;
            Xk(:,number+1)=X_k;           
            number=number+1;           
            %��̬����
            Cnn=[1,-X_k(3),X_k(2);X_k(3),1,-X_k(1);-X_k(2),X_k(1),1];      %p364
            Cnb=Cnn*Cnb;
            Z=AttitudeUpdate(Cnb);         
            Qt=C_Q(Cnb); 
            Zattitude(:,n)=Z;
            Zattitude_IDP(:,m)=Z; 
%             wb=wb-[X_k(10);X_k(11);X_k(12)];
%             fb=fb-[X_k(13);X_k(14);X_k(15)];
            
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
            
            X_k(1:9)=zeros(9,1);
            [x0,y0,z0]= PInCC(P,Rn,e);    
            pwgs_84=[x0;y0;z0]-pwgs0_84;   
            Pwgs_84(:,n)=pwgs_84; 
            Pwgs_84_IDP(:,m)=pwgs_84; 
        
           [Rm,Rn,g]= EarthConditionUpdate(P,Re,e,gideal);         
           [wie,wen]= EarthRotationalVelocityUpdate(P,Rm,Rn,V);   
            
            Wenx(:,m)=wen;
            Wiex(:,m)=wie;
            GG(:,m)=g;
            Rnx(:,m)=Rn;
            Rmx(:,m)=Rm;
            m=m+1;
        end
end

%% ��ͼ
%ZVT������
t_IDP=(1:count); t=t_IDP*Tt;
figure(1)
subplot(3,1,1),plot(t,Zattitude(1,:)/d2r,'b','LineWidth',1),title('Integration Attitude-Yaw'),xlabel('Time (s)'),ylabel('Angle(degree)'),grid on;
subplot(3,1,2),plot(t,Zattitude(2,:)/d2r,'b','LineWidth',1),title('Integration Attitude-Pitch'),xlabel('Time (s)'),ylabel('Angle(degree)'),grid on;
subplot(3,1,3),plot(t,Zattitude(3,:)/d2r,'b','LineWidth',1),title('Integration Attitude-Roll'),xlabel('Time (s)'),ylabel('Angle(degree)'),grid on;

figure(2)
subplot(3,1,1),plot(t,Velocity(1,:),'b','LineWidth',1),title('Velocity  Ve'),xlabel('Time (s)'),ylabel('Velocity (m/s)'),grid on;
subplot(3,1,2),plot(t,Velocity(2,:),'b','LineWidth',1),title('Velocity  Vn'),xlabel('Time (s)'),ylabel('Velocity (m/s)'),grid on;
subplot(3,1,3),plot(t,Velocity(3,:),'b','LineWidth',1),title('Velocity  Vu'),xlabel('Time (s)'),ylabel('Velocity (m/s)'),grid on;

figure(3),
subplot(3,1,1),plot(t,Re*(Position(1,:)-P0(1)),'b','LineWidth',2),title('Latitude'),xlabel('Time (s)'),ylabel('Latitude (m)'),grid on;
subplot(3,1,2),plot(t,Re*(Position(2,:)-P0(2)),'b','LineWidth',2),title('Longtitude'),xlabel('Time (s)'),ylabel('Longtitude (m)'),grid on;
subplot(3,1,3),plot(t,Position(3,:)-P0(3),'b','LineWidth',2),title('Altitude'),xlabel('Time (s)'),ylabel('Altitude (m)'),grid on;

%t_IDP=(1:count/N_step);     
%figure(16)
%plot3(Re*Position_IDP(1,t_IDP),Re*Position_IDP(2,t_IDP),Position_IDP(3,t_IDP),'b','LineWidth',2),title('Longitude-Latitude-Height Changes in INS/GPS','fontsize',11,'fontweight','bold','fontname','Times New Roman'),grid on;
%set(gca,'FontName','Times New Roman','FontSize',9,'fontweight','bold');


