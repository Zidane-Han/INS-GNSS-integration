function [Fdk,Gdk,Hdk,VQk,VRk,line]=KalmanMatrixDiscrete(Ft,Gt,Ht,VQ,VR,Tt,N_step)
            [line,row]=size(Gt);
            [Fdk,Gdk]=c2d(Ft,Gt,Tt*N_step);
            Hdk=Ht;                                 
%             VQt=Gt*VQ*Gt';                           
%             VQk=Tt*VQt+Tt^2/2*((Ft*VQt)'+Ft*VQt);
            VQk=Gdk*VQ*Gdk';
            VRk=VR;