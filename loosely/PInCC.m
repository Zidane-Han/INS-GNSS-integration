function [x0,y0,z0]=PInCC(P,Rn,e)
        x0=(Rn+P(3))*cos(P(1))*cos(P(2));
        y0=(Rn+P(3))*sin(P(2))*cos(P(1));
        z0=(Rn*(1-e^2)+P(3))*sin(P(1));
           