% Datacompensation
%      gideal              	������׼ֵ =9.78049;
%      gn                   ��������ϵ��E-N-U���������ٶȷ�������=[0;0;-1];  
%      Matrix_Wb            E-N-U -> N-E-Dת������=[0 -1 0;-1 0 0;0 0 -1];
%      Matrix_Fb            E-N-U -> N-E-Dת������question��=[0 -1 0;-1 0 0;0 0 -1]; 
%      Sf                   Scale Factor=[0.003,0.003,0.003,0.002,0.002,0.002]
%      Wb                   (3*SampleCounts) the angle velocity of three axis's gyros(unit:deg/s)
%      Fb                   (3*SampleCounts) the accelerated velocity of three axis's accelerometers(unit:gm/s^2)
%      number_IMU_mean      1000;   
%      Wb_BNf�� Fb_BNf                ��̬�ֶ�׼����õ������ݡ��Ӽƾ�ֵ�����������������ʱ��ƫ�ǰ1000�����ݵ�ƽ��ֵ��      
%      wb                   ����/�Ӽ�����Ԥ����ȥ����ֵ��У��...
%      fb                   ...wb:��������ϵ,fb����������ϵ��ǰ������Ϊy�ᣬ���ַ���Ϊx�ᣬ����Ϊ��

function [wb,fb]= DataCompensation(Wbn,Wb_BNf,Fbn,Fb_BNf,Sf,Matrix_Wb,Matrix_Fb,d2r)

         for i=1:3
             wb(i,1)=(Wbn(i)-Wb_BNf(i))/(1+Sf(i));   %Sf �̶�ϵ��
             fb(i,1)=(Fbn(i)-Fb_BNf(i))/(1+Sf(i+3));    
         end
         wb=Matrix_Wb*wb*d2r;
         fb=Matrix_Fb*fb; 