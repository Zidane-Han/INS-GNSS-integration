% MqtGet
% Mqt��qt����

function Mqt=MqtGet(wnb)

        Mqt= [  0       -wnb(1)    -wnb(2)    -wnb(3);
              wnb(1)      0         wnb(3)    -wnb(2);      % �ء��α�P300      
              wnb(2)    -wnb(3)       0        wnb(1);
              wnb(3)     wnb(2)    -wnb(1)       0     ];
        
  