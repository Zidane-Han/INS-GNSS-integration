% EarthRotationalVelocityUpdate
%      wie                ������ת���ٶȣ�E-N-U
%      wen                λ�����ʣ�E-N-U

function [wie,wen]=EarthRotationalVelocityUpdate(P,Rm,Rn,V)
  wie=degtorad(15.041088)/3600*[0;cos(P(1));sin(P(1))];                   
  wen=[-V(2)/(Rm);V(1)/(Rn);tan(P(1))*V(1)/(Rn)];    %�� �α�page300