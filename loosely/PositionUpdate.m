function P=PositionUpdate(P,V,oldV,Tt,Rm,Rn)

        latitude=P(1)+0.5*(V(1)+oldV(1))*Tt/((Rn+P(3))*cos(P(1)));
        longtitude=P(2)+0.5*(V(2)+oldV(2))*Tt/(Rm+P(3));
        altitude=P(3)+0.5*(V(3)+oldV(3))*Tt;
        P=[latitude;longtitude;altitude];