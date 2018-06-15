% EarthConditionUpdate
%      Rm                ����Ȧ�����ʰ뾶
%      Rn                î��Ȧ�����ʰ뾶
function [Rm,Rn,g]=EarthConditionUpdate(P,Re,e,gideal)
        Rm=Re*(1-2*e+3*e*(sin(P(1)))^2);           
        Rn=Re*(1+e*(sin(P(1)))^2);                       
        g=gideal*(1-2*P(3)/Re); %P213
  