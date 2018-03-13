% genPos_ecef
function Pos_ecef = genPecef(RmRn,e,Pos_n2)

    Pos_ecef(1,:) = (RmRn(2,:)+Pos_n2(3,:))*cos(Pos_n2(1,:))*cos(Pos_n2(2,:));   % ECEF
    Pos_ecef(2,:) = (RmRn(2,:)+Pos_n2(3,:))*cos(Pos_n2(1,:))*sin(Pos_n2(2,:));
    Pos_ecef(3,:) = (RmRn(2,:)*(1-e^2)+Pos_n2(3,:))*sin(Pos_n2(1,:)); 