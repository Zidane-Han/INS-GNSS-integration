% EarthRotationalVelocityUpdate
%      wie                地球自转角速度，E-N-U
%      wen                位置速率，E-N-U

function [wie,wen]=EarthRotationalVelocityUpdate(P,Rm,Rn,V)
  wie=degtorad(15.041088)/3600*[0;cos(P(1));sin(P(1))];                   
  wen=[-V(2)/(Rm);V(1)/(Rn);tan(P(1))*V(1)/(Rn)];    %秦 课本page300