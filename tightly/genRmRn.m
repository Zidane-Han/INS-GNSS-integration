% genRmRn
function RmRn = genRmRn(Re,f,e,Pos_n2)

RmRn(1) = Re*(1-2*f+3*f*sin(Pos_n2(1))^2);  % Rm
RmRn(2) = Re*(1+f*(sin(Pos_n2(1)))^2);      % Rn