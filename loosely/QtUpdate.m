function Qt=QtUpdate(oldMqt,Mqt,Tt,Qt)
        K1= 1/2*oldMqt*Qt*Tt;
        K2= 1/4*(oldMqt+Mqt)*(Qt+1/2*K1)*Tt; 
        K3= 1/4*(oldMqt+Mqt)*(Qt+1/2*K2)*Tt;
        K4= 1/2*Mqt*(Qt+K3)*Tt;
        Qt= Qt+(K1+2*K2+2*K3+K4)/6;             
                   
        modQ=sqrt(Qt(1)^2+Qt(2)^2+Qt(3)^2+Qt(4)^2); %¹æ·¶»¯
        Qt=Qt/modQ;    