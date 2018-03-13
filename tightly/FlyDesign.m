%% FlyDesign 

clear all
close all

%--------------------------------------------------------------------------
%% ��������ϵENU�·��й켣�����
t0=3600;           %���з�����ʱ��;
Time=[t0];
d2r=pi/180;
initvel=1;        %�����ٶȣ���λ����գ�
initPos = [30*d2r;120*d2r;5000];


for i=1:1:t0      %ȫ����
    btheta(i)=0;  %���帩���ǣ������������������Ľǹ�ϵ��  
    gama(i)=0;    %��������
    fai(i)=0;     %���庽���
    theta(i)=0;   %������ǣ������������������Ľǹ�ϵ��
    faiv(i)=0;    %����ƫ��
    beta(i)=0;    %����⻬�ǣ������������ٶ�����Ľǹ�ϵ��
    arfa(i)=0;    %���幥��
    gamav(i)=0;   %�����ٶ���б�ǣ������������ٶ�����Ľǹ�ϵ��   
    
    ma(i)=initvel; 
end

for i=1:1:t0 
    
    AngleDesign(1,i)=btheta(i)*d2r;  % rad  
    AngleDesign(2,i)=gama(i)*d2r;        
    AngleDesign(3,i)=fai(i)*d2r;
    AngleDesign(4,i)=theta(i)*d2r;
    AngleDesign(5,i)=faiv(i)*d2r;
    AngleDesign(6,i)=beta(i)*d2r;    
    AngleDesign(7,i)=arfa(i)*d2r;
    AngleDesign(8,i)=gamav(i)*d2r;   
    
    VelocityDesign(1,i)=ma(i)*340;    % m/s
end
%--------------------------------------------------------------------------
%% ���浯���켣����
save('Time');
save('AngleDesign');
save('VelocityDesign');
save('initPos');
