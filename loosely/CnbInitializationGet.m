% CnbInitializationGet
%      Cnb                  ��̬����E-N-U
%      Qt                   ��Ԫ����E-N-U
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Cnb,Qt]=CnbInitializationGet(Z)
     yaw=Z(1);                          
     pitch=Z(2);                         
     roll=Z(3);    
     %page 297/9.2.39 
     Cnb(1,1)= cos(roll)*cos(yaw)+sin(roll)*sin(pitch)*sin(yaw);
     Cnb(2,1)=-cos(roll)*sin(yaw)+sin(roll)*sin(pitch)*cos(yaw);
     Cnb(3,1)=-sin(roll)*cos(pitch);
     Cnb(1,2)= cos(pitch)*sin(yaw);
     Cnb(2,2)= cos(pitch)*cos(yaw);
     Cnb(3,2)= sin(pitch);
     Cnb(1,3)= sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw);
     Cnb(2,3)=-sin(roll)*sin(yaw)-cos(roll)*sin(pitch)*cos(yaw);
     Cnb(3,3)= cos(roll)*cos(pitch);
     Cnb;
    
     %page304/9.2.63 ��Ԫ����ֵQ(0)�ɽ����ߵ��ĳ�ʼ��׼ȷ��
     %Qt���ŵ�ȷ������Ҫ���ǣ�����
     
     %��һ��QT���㹫ʽ��
%      Qt(1,1)=0.5*sqrt(1+Cnb(1,1)+Cnb(2,2)+Cnb(3,3));
%      Qt(2,1)=0.5*sqrt(1+Cnb(1,1)-Cnb(2,2)-Cnb(3,3));
%      Qt(3,1)=0.5*sqrt(1-Cnb(1,1)+Cnb(2,2)-Cnb(3,3));
%      Qt(4,1)=0.5*sqrt(1-Cnb(1,1)-Cnb(2,2)+Cnb(3,3));
     
     Qt(1,1)=0.5*sqrt(1+Cnb(1,1)+Cnb(2,2)+Cnb(3,3));  
     Qt(2,1)=0.25*(Cnb(3,2)-Cnb(2,3))/Qt(1,1);
     Qt(3,1)=0.25*(Cnb(1,3)-Cnb(3,1))/Qt(1,1);
     Qt(4,1)=0.25*(Cnb(2,1)-Cnb(1,2))/Qt(1,1);
     modQt=sqrt(Qt(1,1)^2+Qt(2,1)^2+Qt(3,1)^2+Qt(4,1)^2);
     Qt=Qt/modQt; 