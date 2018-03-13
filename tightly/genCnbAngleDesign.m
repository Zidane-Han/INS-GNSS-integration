% genCnb_AngleDesign
function Cnb_AngleDesign = genCnb_AngleDesign(AngleDesign)

Cnb_AngleDesign(1,1) =  sin(AngleDesign(3,:))*sin(AngleDesign(2,:))*sin(AngleDesign(1,:)) + cos(AngleDesign(3,:))*cos(AngleDesign(2,:));
Cnb_AngleDesign(1,2) =  cos(AngleDesign(3,:))*sin(AngleDesign(2,:))*sin(AngleDesign(1,:)) - sin(AngleDesign(3,:))*cos(AngleDesign(2,:));
Cnb_AngleDesign(1,3) = -cos(AngleDesign(1,:))*sin(AngleDesign(2,:));

Cnb_AngleDesign(2,1) =  sin(AngleDesign(3,:))*cos(AngleDesign(1,:));
Cnb_AngleDesign(2,2) =  cos(AngleDesign(3,:))*cos(AngleDesign(1,:));
Cnb_AngleDesign(2,3) =  sin(AngleDesign(1,:));

Cnb_AngleDesign(3,1) =  cos(AngleDesign(3,:))*sin(AngleDesign(2,:))-sin(AngleDesign(3,:))*sin(AngleDesign(1,:))*cos(AngleDesign(2,:));
Cnb_AngleDesign(3,2) = -sin(AngleDesign(3,:))*sin(AngleDesign(2,:))-cos(AngleDesign(3,:))*sin(AngleDesign(1,:))*cos(AngleDesign(2,:));
Cnb_AngleDesign(3,3) =  cos(AngleDesign(2,:))*cos(AngleDesign(1,:));


% Cnb_AngleDesign(1,1) =  cos(AngleDesign(3,:))*cos(AngleDesign(2,:));
% Cnb_AngleDesign(1,2) =  sin(AngleDesign(2,:));
% Cnb_AngleDesign(1,3) =  sin(AngleDesign(3,:))*cos(AngleDesign(2,:));
% 
% Cnb_AngleDesign(2,1) = -sin(AngleDesign(3,:))*sin(AngleDesign(1,:))-cos(AngleDesign(3,:))*sin(AngleDesign(2,:))*cos(AngleDesign(1,:));
% Cnb_AngleDesign(2,2) =  cos(AngleDesign(2,:))*cos(AngleDesign(1,:));
% Cnb_AngleDesign(2,3) =  cos(AngleDesign(3,:))*sin(AngleDesign(1,:))-sin(AngleDesign(3,:))*sin(AngleDesign(2,:))*cos(AngleDesign(1,:));
% 
% Cnb_AngleDesign(3,1) =  cos(AngleDesign(3,:))*sin(AngleDesign(2,:))*sin(AngleDesign(1,:)) - sin(AngleDesign(3,:))*cos(AngleDesign(1,:));
% Cnb_AngleDesign(3,2) = -cos(AngleDesign(2,:))*sin(AngleDesign(1,:));
% Cnb_AngleDesign(3,3) =  sin(AngleDesign(3,:))*sin(AngleDesign(2,:))*sin(AngleDesign(1,:)) + cos(AngleDesign(3,:))*cos(AngleDesign(1,:));