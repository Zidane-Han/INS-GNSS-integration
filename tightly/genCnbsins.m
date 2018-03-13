% genCnbsins
%%1 function Cnb_sins = genCnbsins(Q)
% 
% Cbn_sins(1,1) =  Q(1)^2 + Q(2)^2 - Q(3)^2 - Q(4)^2;
% Cbn_sins(1,2) =  2*(Q(2)*Q(3) - Q(1)*Q(4));
% Cbn_sins(1,3) =  2*(Q(2)*Q(4) + Q(1)*Q(3));
% Cbn_sins(2,1) =  2*(Q(2)*Q(3) + Q(1)*Q(4));
% Cbn_sins(2,2) =  Q(1)^2 - Q(2)^2 + Q(3)^2 - Q(4)^2;
% Cbn_sins(2,3) =  2*(Q(3)*Q(4) - Q(1)*Q(2));
% Cbn_sins(3,1) =  2*(Q(2)*Q(4) - Q(1)*Q(3));
% Cbn_sins(3,2) =  2*(Q(3)*Q(4) + Q(1)*Q(2));
% Cbn_sins(3,3) =  Q(1)^2 - Q(2)^2 - Q(3)^2 + Q(4)^2;
% 
% Cnb_sins = Cbn_sins';

function Cnb_sins = genCnbsins(Attitude_sins)

pitch = Attitude_sins(1);  %rad
roll = Attitude_sins(2);
yaw = Attitude_sins(3);

Cnb_sins=[sin(roll)*sin(pitch)*sin(yaw)+cos(roll)*cos(yaw) sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw) -sin(roll)*cos(pitch);
          cos(pitch)*sin(yaw) cos(pitch)*cos(yaw) sin(pitch);
          sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw) -sin(roll)*sin(yaw)-cos(roll)*sin(pitch)*cos(yaw) cos(roll)*cos(pitch)];
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              