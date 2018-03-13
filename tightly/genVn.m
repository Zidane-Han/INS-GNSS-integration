% genV_b
function Vel_n = genV_n(VelocityDesign,AngleDesign)

Vel_n(1,:) = VelocityDesign(1,:) * cos(AngleDesign(4,:)) * sin(AngleDesign(5,:));
Vel_n(2,:) = VelocityDesign(1,:) * cos(AngleDesign(4,:)) * cos(AngleDesign(5,:));
Vel_n(3,:) = VelocityDesign(1,:) * sin(AngleDesign(4,:));
