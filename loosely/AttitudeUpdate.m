function Z=AttitudeUpdate(Cnb)
        %Z(1) - º½Ïò½Ç ÇØ¿Î±¾page297/±í9-2-1
        if  abs(Cnb(2,2))<= eps && Cnb(1,2)>0
            Z(1)=1/2*pi;
        elseif abs(Cnb(2,2))<= eps && Cnb(1,2)<0 
            Z(1)=-1/2*pi;
        elseif Cnb(2,2)>0 
               Z(1)=atan(Cnb(1,2)/Cnb(2,2)); 
        elseif Cnb(2,2)<0 && Cnb(1,2)>0
               Z(1)=atan(Cnb(1,2)/Cnb(2,2))+pi; 
        elseif Cnb(2,2)<0 && Cnb(1,2)<0
               Z(1)=atan(Cnb(1,2)/Cnb(2,2))-pi; 
        end
        
        %Z(2) - ¸©Ñö½Ç ÇØ¿Î±¾page297/0.2.41
       
         Z(2)=asin(Cnb(3,2));  
        
        %Z(3) - ºá¹ö½Ç ÇØ¿Î±¾page297/±í9-2-1
        if  Cnb(3,3)>0
            Z(3)=atan(-Cnb(3,1)/(Cnb(3,3)+eps));             
        elseif Cnb(3,3)<0 && Cnb(3,1)>0 
            Z(3)=atan(-Cnb(3,1)/(Cnb(3,3)+eps))-pi;
        elseif Cnb(3,3)<0 && Cnb(3,1)<0 
            Z(3)=atan(-Cnb(3,1)/(Cnb(3,3)+eps))+pi;
        end
        
        Z=[Z(1);Z(2);Z(3)];