% genCmpV
function CmpV = genCmpV(wien,wenn,Venu_sins)

CmpW = 2*wien + wenn;

CmpV = [0 -CmpW(3) CmpW(2);
        CmpW(3) 0 -CmpW(1);
        -CmpW(2) CmpW(1) 0]*Venu_sins;
