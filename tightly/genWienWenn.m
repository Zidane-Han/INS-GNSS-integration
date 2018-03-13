%% genWienWenn
function [Rm,Rn,wien,wenn] = genWienWenn(Penu2_sins,Venu_sins)

d2r = pi/180;
Wie = 15/3600*d2r;
Re=6378137;
f=1/298.257223563;
e=sqrt(0.00669437999013);  

Rm = Re*(1-2*f+3*f*sin(Penu2_sins(1,:))^2);  % Rm
Rn = Re*(1+f*(sin(Penu2_sins(1,:)))^2);      % Rn

wien = [0;
        Wie*cos(Penu2_sins(1,:));
        Wie*sin(Penu2_sins(1,:))];
wenn = [-Venu_sins(2,:)/(Rm + Penu2_sins(3,:));
         Venu_sins(1,:)/(Rn + Penu2_sins(3,:));
         Venu_sins(1,:)/(Rn + Penu2_sins(3,:)) * tan(Penu2_sins(1,:))];  % rad