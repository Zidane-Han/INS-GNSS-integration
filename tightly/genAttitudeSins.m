% genAttitudeSins

function Attitude_sins = genAttitudeSins(Cnbsins)

% if Cnbsins(3,3)>0
%    Attitude_sins(1,:) = atan(-Cnbsins(1,3)/(Cnbsins(3,3)+eps));      % RollSins
% elseif Cnbsins(3,3)<0 && Cnbsins(1,3)>0
%    Attitude_sins(1,:) = atan(-Cnbsins(1,3)/(Cnbsins(3,3)+eps)) - pi;  
% elseif Cnbsins(3,3)<0 && Cnbsins(1,3)<0
%    Attitude_sins(1,:) = atan(-Cnbsins(1,3)/(Cnbsins(3,3)+eps)) + pi;
% else
%     error('"Cnbsins" may have an  error value');   
% end
% 
% Attitude_sins(2,:) = asin(Cnbsins(2,3));                              % PitchSins

if Cnbsins(3,3)>0
   Attitude_sins(2,:) = atan(-Cnbsins(1,3)/(Cnbsins(3,3)+eps));      % RollSins
elseif Cnbsins(3,3)<0 && Cnbsins(1,3)>0
   Attitude_sins(2,:) = atan(-Cnbsins(1,3)/(Cnbsins(3,3)+eps)) - pi;  
elseif Cnbsins(3,3)<0 && Cnbsins(1,3)<0
   Attitude_sins(2,:) = atan(-Cnbsins(1,3)/(Cnbsins(3,3)+eps)) + pi;
else
    error('"Cnbsins" may have an  error value');   
end

Attitude_sins(1,:) = asin(Cnbsins(2,3));                              % PitchSins

if abs(Cnbsins(2,2))<=eps && Cnbsins(2,1)>0                           % YawSins
   Attitude_sins(3,:) = pi/2;
elseif abs(Cnbsins(2,2))<=eps && Cnbsins(2,1)<0
   Attitude_sins(3,:) = -pi/2;
elseif Cnbsins(2,2)>0
   Attitude_sins(3,:) = atan(Cnbsins(2,1)/(Cnbsins(2,2)+eps));      
elseif Cnbsins(2,2)<0 && Cnbsins(2,1)>0
   Attitude_sins(3,:) = atan(Cnbsins(2,1)/(Cnbsins(2,2)+eps)) + pi;
elseif Cnbsins(2,2)<0 && Cnbsins(2,1)<0
   Attitude_sins(3,:) = atan(Cnbsins(2,1)/(Cnbsins(2,2)+eps)) - pi;  
end


