% MqtGet
% Mqt£ºqtµ¼Êý

function Mqt=MqtGet(wnb)

        Mqt= [  0       -wnb(1)    -wnb(2)    -wnb(3);
              wnb(1)      0         wnb(3)    -wnb(2);      % ÇØ¡ª¿Î±¾P300      
              wnb(2)    -wnb(3)       0        wnb(1);
              wnb(3)     wnb(2)    -wnb(1)       0     ];
        
  