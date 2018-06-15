function [Ft,Gt,Ht]=KalmanMatrix(P,V,Rm,Rn,wie,fn,Cnb)
            tanphi=tan(P(1));                            
            cosphi=cos(P(1));
            secphi=1.0/cosphi;
            sinphi=sin(P(1));
            cosphi=cos(P(1));
            Rnh=1.0/(Rn+P(3));
            Rmh=1.0/(Rm+P(3));
            
            %ÇØ ¿Î±¾page360
            Fins(1,2)= (wie(3)+V(1)*tanphi*Rnh);
            Fins(1,3)=-(wie(2)+V(1)*Rnh);
            Fins(1,5)=- Rmh;   
            Fins(1,9)= V(2)*(Rmh^2); 
            
            Fins(2,1)=-(wie(3)+V(1)*tanphi*Rnh);
            Fins(2,3)=-V(2)*Rmh;
            Fins(2,4)= Rnh;
            Fins(2,7)=-wie(3);
            Fins(2,9)=-V(1)*(Rnh^2);
            
            Fins(3,1)= wie(2)+V(1)*Rnh;     %wie(2)=wie*cosL
            Fins(3,2)= V(2)*Rmh;
            Fins(3,4)= tanphi*Rnh;
            Fins(3,7)= wie(2)+V(1)*(secphi)^2*Rnh;
            Fins(3,9)=-V(1)*tanphi*(Rnh)^2;
            
            Fins(4,2)=-fn(3);
            Fins(4,3)= fn(2);
            Fins(4,4)= (V(2)*tanphi-V(3))*Rnh;
            Fins(4,5)= (2*wie(3)+V(1)*tanphi*Rnh);
            Fins(4,6)=-(2*wie(2)+V(1)*Rnh);
            Fins(4,7)= 2*wie(2)*V(2)+V(1)*V(2)*(secphi)^2*Rnh+2*wie(3)*V(3);
            Fins(4,9)= (V(1)*V(3)-V(1)*V(2)*tanphi)*(Rnh^2);
            
            Fins(5,1)= fn(3);
            Fins(5,3)=-fn(1);
            Fins(5,4)=-2*(wie(3)+V(1)*tanphi*Rnh);
            Fins(5,5)=-V(3)*Rmh;
            Fins(5,6)=-V(2)*Rmh;
            Fins(5,7)=-(2*wie(2)*V(1)+V(1)^2*(secphi)^2*Rnh);
            Fins(5,9)= V(1)^2*tanphi*(Rnh^2)+V(2)*V(3)*(Rmh^2);
            
            Fins(6,1)=-fn(2);
            Fins(6,2)= fn(1);
            Fins(6,4)= 2*(wie(2)+V(1)*Rnh);
            Fins(6,5)= 2*V(2)*Rmh;
            Fins(6,7)=-2*wie(3)*V(1);
            Fins(6,9)=-(V(2)^2*(Rmh^2)+V(1)^2*(Rnh^2));
                        
            Fins(8,4)= Rnh*secphi;    Fins(8,7)= V(1)*secphi*tanphi*Rnh;
            Fins(8,9)=-V(1)*secphi*(Rnh^2);  
            
            Fins(7,5)= Rmh;   Fins(7,9)=-V(2)*(Rmh^2); 
            Fins(9,6)= 1;

%             T=[zeros(3,6);zeros(3,3),-Cnb;zeros(3,6)];
%             Ft=[Fins,T;Cnb,zeros(3,12);zeros(3,15)];
            T=[-Cnb,zeros(3,3);zeros(3,3),Cnb;zeros(3,6)];
            Ft=[Fins,T;zeros(6,15)];

%             Gt=[Cnb,zeros(3,3);zeros(3,3),Cnb;zeros(3,6);eye(6)];
            Gt=[Cnb,zeros(3,3);zeros(3,3),Cnb;zeros(3,6);zeros(6,6)];

            Ht=[zeros(3,6),eye(3,3),zeros(3,6);
                zeros(3,3),eye(3,3),zeros(3,9)];
