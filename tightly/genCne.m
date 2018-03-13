% genCne
function Cne = genCne(Pos_n2)

    Cne(:,:)=[-sin(Pos_n2(2,:)) -sin(Pos_n2(1,:))*cos(Pos_n2(2,:)) cos(Pos_n2(1,:))*cos(Pos_n2(2,:));
               cos(Pos_n2(2,:)) -sin(Pos_n2(1,:))*sin(Pos_n2(2,:)) cos(Pos_n2(1,:))*sin(Pos_n2(2,:));
                 0 cos(Pos_n2(1,:)) sin(Pos_n2(1,:))];