%% Integration

load('Time');
load('Wibb');load('errDriftGyro');load('BiasGyro');load('WhiteGyro');load('kg');  % [rad;rad;rad]
load('Fb');load('errDriftAccel');load('BiasAccel');load('WhiteAccel');load('ka'); % [m/s^2;m/s^2;m/s^2]           
load('RmRn');            % [Rm(m); Rn(m)]

load('Pos_n2_real');     % [Lat(rad); Lon(rad); Hei(m)]
load('Vel_n_real');      % [m/s; m/s;m/s]
load('Attitude_real');   % [Roll(rad); Pitch(rad); Yaw(rad)]
load('Cnb_real');
load('Cne_real');
load('Pos_ecef_real');   % [m; m;m]
load('Vel_ecef_real');   % [m/s; m/s;m/s]

load('GNSS1Pos');load('GNSS2Pos');load('GNSS3Pos');load('GNSS4Pos');
load('GNSS1V');load('GNSS2V');load('GNSS3V');load('GNSS4V');
load('Pseudo_gps');load('BiasTu');load('PseudoNoise_Value');load('MeasPseudoNoise_Value');
load('PseudoRate_gps');load('BiasTru');load('PseudoRateNoise_Value');load('MeasPseudoRateNoise_Value');
%--------------------------------------------------------------------------
g = 9.8;                 
d2r = pi/180;
Wie = 15/3600*d2r;
fg = [0;0;-g];
c=3.0e+8;
tstep = 1;

%% ¬À≤®◊¥Ã¨≥ı÷µ
PKF = [(1e-5)^2,(1e-5)^2,(15)^2,(1)^2,(1)^2,(1)^2,(0.01)^2,(0.01)^2,(0.01)^2,(BiasGyro)^2,(BiasGyro)^2,(BiasGyro)^2,(BiasAccel)^2,(BiasAccel)^2,(BiasAccel)^2,(3)^2,(0.03)^2];
QKF = horzcat(diag([(WhiteGyro)^2;(WhiteGyro)^2;(WhiteGyro)^2;(WhiteAccel)^2;(WhiteAccel)^2;(WhiteAccel)^2;3^2;(0.03)^2]));
RKF = [3^2,3^2,3^2,3^2,0.03^2,0.03^2,0.03^2,0.03^2];

for i=1:Time(1)
    
%% SINSΩ‚À„
    if i==1   
       Venu_sins(:,1) = Vel_n_real(:,1);
       Penu1_sins(:,1) = Pos_n1_real(:,1);
       Penu2_sins(:,1) = Pos_n2_real(:,1);
       Attitude_sins(:,1) = Attitude_real(:,1);
       Cnb_sins(:,:,1) = Cnb_real(:,:,1);
       
       [Rm(1),Rn(1),wien(:,1),wenn(:,1)] = genWienWenn(Penu2_sins(:,1),Venu_sins(:,1)); 
       wnbbsins(:,1) = Wibb(:,1)-Cnb_sins(:,:,1)*(wien(:,1)+wenn(:,1));  
       CmpV(:,1) = [0;0;0];    
       
       Pecef_sins(:,1) = Pos_ecef_real(:,1);
       
       deltaEvaluBG(:,1) = [BiasGyro;BiasGyro;BiasGyro];
       deltaEvaluBA(:,1) = [BiasAccel;BiasAccel;BiasAccel];
       fn = Cnb_sins(:,:,i)'*(Fb(:,i)-deltaEvaluBA(:,1));
    else     
       [Rm(i),Rn(i),wien(:,i),wenn(:,i)] = genWienWenn(Penu2_sins(:,i-1),Venu_sins(:,i-1));  % ◊ÀÃ¨Ω‚À„ 
       wnbbsins(:,i) = Wibb(:,i) - Cnb_sins(:,:,i-1)*(wien(:,i)+wenn(:,i)) - deltaEvaluBG(:,i-1);    
       Attitude_sins(:,i) = [cos(Attitude_sins(2,i-1))*wnbbsins(1,i) + sin(Attitude_sins(2,i-1))*wnbbsins(3,i) + Attitude_sins(1,i-1);
                             sin(Attitude_sins(2,i-1))*tan(Attitude_sins(1,i-1))*wnbbsins(1,i)+ wnbbsins(2,i) - cos(Attitude_sins(2,i-1))*tan(Attitude_sins(1,i-1))*wnbbsins(3,i) + Attitude_sins(2,i-1);
                             1/cos(Attitude_sins(1,i-1))*(sin(Attitude_sins(2,i-1))*wnbbsins(1,i)-cos(Attitude_sins(2,i-1))*wnbbsins(3,i)) + Attitude_sins(3,i-1)];
       Cnb_sins(:,:,i) = genCnbsins(Attitude_sins(:,i));
       
       CmpV(:,i) = genCmpV(wien(:,i),wenn(:,i),Venu_sins(:,i-1));                            % ÀŸ∂»Ω‚À„
       Venu_sins(:,i) = Venu_sins(:,i-1) + Cnb_sins(:,:,i)'*(Fb(:,i)-deltaEvaluBA(:,i-1)) - CmpV(:,i) + fg; 
       fn = Cnb_sins(:,:,i)'*(Fb(:,i)-deltaEvaluBA(:,i-1));
       
       Penu1_sins(:,i) = Penu1_sins(:,i-1) + Venu_sins(:,i)*t;                               % Œª÷√Ω‚À„                              
       Penu2_sins(1,i) = Penu2_sins(1,i-1) + Venu_sins(2,i)/(Rm(i) + Penu2_sins(3,i-1));                          
       Penu2_sins(2,i) = Penu2_sins(2,i-1) + Venu_sins(1,i)/((Rn(i) + Penu2_sins(3,i-1))*cos(Penu2_sins(1,i)));
       Penu2_sins(3,i) = Penu2_sins(3,i-1) + Venu_sins(3,i);  

       Pecef_sins(:,i) = [(Rn(i) + Penu2_sins(3,i))*cos(Penu2_sins(1,i))*cos(Penu2_sins(2,i));
                          (Rn(i) + Penu2_sins(3,i))*cos(Penu2_sins(1,i))*sin(Penu2_sins(2,i));
                          (Rn(i)*(1-e^2) + Penu2_sins(3,i))*sin(Penu2_sins(1,i))]; 
    end     
       
       Cne_sins(:,:,i) = [-sin(Penu2_sins(2,i)) -sin(Penu2_sins(1,i))*cos(Penu2_sins(2,i)) cos(Penu2_sins(1,i))*cos(Penu2_sins(2,i));
                           cos(Penu2_sins(2,i)) -sin(Penu2_sins(1,i))*sin(Penu2_sins(2,i)) cos(Penu2_sins(1,i))*sin(Penu2_sins(2,i));
                           0 cos(Penu2_sins(1,i)) sin(Penu2_sins(1,i))];
       Vecef_sins(:,i) = Cne_sins(:,:,i)*Venu_sins(:,i); 
    
%% INSŒ±æ‡/Œ±æ‡¬ º∆À„
    Pseudo_sins(1,i) = sqrt((Pecef_sins(1,i)-GNSS1Pos(i,1))^2+(Pecef_sins(2,i)-GNSS1Pos(i,2))^2+(Pecef_sins(3,i)-GNSS1Pos(i,3))^2);
    Pseudo_sins(2,i) = sqrt((Pecef_sins(1,i)-GNSS2Pos(i,1))^2+(Pecef_sins(2,i)-GNSS2Pos(i,2))^2+(Pecef_sins(3,i)-GNSS2Pos(i,3))^2);
    Pseudo_sins(3,i) = sqrt((Pecef_sins(1,i)-GNSS3Pos(i,1))^2+(Pecef_sins(2,i)-GNSS3Pos(i,2))^2+(Pecef_sins(3,i)-GNSS3Pos(i,3))^2);
    Pseudo_sins(4,i) = sqrt((Pecef_sins(1,i)-GNSS4Pos(i,1))^2+(Pecef_sins(2,i)-GNSS4Pos(i,2))^2+(Pecef_sins(3,i)-GNSS4Pos(i,3))^2);
    
    PseudoRate_sins(1,i) = ((Pecef_sins(1,i)-GNSS1Pos(i,1))*(Vecef_sins(1,i)-GNSS1V(i,1)) + (Pecef_sins(2,i)-GNSS1Pos(i,2))*(Vecef_sins(2,i)-GNSS1V(i,2)) + (Pecef_sins(3,i)-GNSS1Pos(i,3))*(Vecef_sins(3,i)-GNSS1V(i,3))) / Pseudo_sins(1,i);
    PseudoRate_sins(2,i) = ((Pecef_sins(1,i)-GNSS2Pos(i,1))*(Vecef_sins(1,i)-GNSS2V(i,1)) + (Pecef_sins(2,i)-GNSS2Pos(i,2))*(Vecef_sins(2,i)-GNSS2V(i,2)) + (Pecef_sins(3,i)-GNSS2Pos(i,3))*(Vecef_sins(3,i)-GNSS2V(i,3))) / Pseudo_sins(2,i);
    PseudoRate_sins(3,i) = ((Pecef_sins(1,i)-GNSS3Pos(i,1))*(Vecef_sins(1,i)-GNSS3V(i,1)) + (Pecef_sins(2,i)-GNSS3Pos(i,2))*(Vecef_sins(2,i)-GNSS3V(i,2)) + (Pecef_sins(3,i)-GNSS3Pos(i,3))*(Vecef_sins(3,i)-GNSS3V(i,3))) / Pseudo_sins(3,i);
    PseudoRate_sins(4,i) = ((Pecef_sins(1,i)-GNSS4Pos(i,1))*(Vecef_sins(1,i)-GNSS4V(i,1)) + (Pecef_sins(2,i)-GNSS4Pos(i,2))*(Vecef_sins(2,i)-GNSS4V(i,2)) + (Pecef_sins(3,i)-GNSS4Pos(i,3))*(Vecef_sins(3,i)-GNSS4V(i,3))) / Pseudo_sins(4,i);
       
%% ¡ø≤‚Œ±æ‡/Œ±æ‡¬ º∆À„
    deltaPseudo(:,i) = Pseudo_sins(:,i)- Pseudo_gps(1:4,i); 
    deltaPseudoRate(:,i) = PseudoRate_sins(:,i) - PseudoRate_gps(1:4,i);  
    
    Zkf(:,i) = vertcat(deltaPseudo(:,i),deltaPseudoRate(:,i));
    Z = Zkf(:,i);
    
%% GPS/INSΩÙ◊È∫œ
% ◊¥Ã¨æÿ’Û
    F1(:,:,i) = [zeros(1,2) -Venu_sins(2,i)/(Rm(i)+Penu2_sins(3,i))^2 0 1/(Rm(i)+Penu2_sins(3,i)) zeros(1,12);
                 Venu_sins(1,i)*sec(Penu2_sins(1,i))*tan(Penu2_sins(1,i))/(Rn(i)+Penu2_sins(3,i)) 0 -Venu_sins(1,i)*sec(Penu2_sins(1,i))/(Rn(i)+Penu2_sins(3,i))^2 sec(Penu2_sins(1,i))/(Rn(i)+Penu2_sins(3,i)) zeros(1,13);
                 zeros(1,5) 1 zeros(1,11)];
    F2111(:,i) =  2*Wie*(Venu_sins(3,i)*sin(Penu2_sins(1,i))+Venu_sins(2,i)*cos(Penu2_sins(1,i))) + Venu_sins(1,i)*Venu_sins(2,i)*sec(Penu2_sins(1,i))^2/(Rn(i)+Penu2_sins(3,i));
    F2113(:,i) =  (Venu_sins(1,i)*Venu_sins(3,i) - Venu_sins(1,i)*Venu_sins(2,i)*tan(Penu2_sins(1,i)))/(Rn(i)+Penu2_sins(3,i))^2;
    F2114(:,i) =  (Venu_sins(2,i)*tan(Penu2_sins(1,i))-Venu_sins(3,i))/(Rn(i)+Penu2_sins(3,i));
    F2115(:,i) =  2*Wie*sin(Penu2_sins(1,i)) + Venu_sins(1,i)*tan(Penu2_sins(1,i))/(Rn(i)+Penu2_sins(3,i));
    F2116(:,i) = -(2*Wie*cos(Penu2_sins(1,i))+Venu_sins(1,i)/(Rn(i)+Penu2_sins(3,i)));
    F2121(:,i) = -(2*Venu_sins(1,i)*Wie*cos(Penu2_sins(1,i)) + (Venu_sins(1,i)*sec(Penu2_sins(1,i)))^2/(Rn(i)+Penu2_sins(3,i)));
    F2123(:,i) =  Venu_sins(2,i)*Venu_sins(3,i)/(Rm(i)+Penu2_sins(3,i))^2 + Venu_sins(1,i)^2*tan(Penu2_sins(1,i))/(Rn(i)+Penu2_sins(3,i))^2;
    F2124(:,i) = -2*(Wie*sin(Penu2_sins(1,i))+Venu_sins(1,i)*tan(Penu2_sins(1,i))/(Rn(i)+Penu2_sins(3,i)));
    F2125(:,i) = -Venu_sins(3,i)/(Rm(i)+Penu2_sins(3,i));
    F2126(:,i) = -Venu_sins(2,i)/(Rm(i)+Penu2_sins(3,i));
    F2131(:,i) = -2*Wie*Venu_sins(1,i)*sin(Penu2_sins(1,i));
    F2133(:,i) = -((Venu_sins(2,i)/(Rm(i)+Penu2_sins(3,i)))^2 + (Venu_sins(1,i)/(Rn(i)+Penu2_sins(3,i)))^2);
    F2134(:,i) = 2*(Wie*cos(Penu2_sins(1,i))+Venu_sins(1,i)/(Rn(i)+Penu2_sins(3,i)));
    F2135(:,i) = 2*Venu_sins(2,i)/(Rm(i)+Penu2_sins(3,i));
    F21(:,:,i) = [ F2111(:,i) 0 F2113(:,i) F2114(:,i) F2115(:,i) F2116(:,i);
                   F2121(:,i) 0 F2123(:,i) F2124(:,i) F2125(:,i) F2126(:,i);
                   F2131(:,i) 0 F2133(:,i) F2134(:,i) F2135(:,i) 0];    
                        
    F22(:,:,i) = [ 0 -fn(3) fn(2); 
                   fn(3) 0 -fn(1);
                  -fn(2) fn(1) 0];
    F23(:,:,i) = zeros(3,3);          
    F24(:,:,i) = zeros(3,2);
    F2(:,:,i) = horzcat(F21(:,:,i),F22(:,:,i),F23(:,:,i),Cnb_sins(:,:,i)',F24(:,:,i));
    F31(:,:,i) = [ 0 0 Venu_sins(2,i)/(Rm(i)+Penu2_sins(3,i))^2;
                  -Wie*sin(Penu2_sins(1,i)) 0 -Venu_sins(1,i)/(Rn(i)+Penu2_sins(3,i))^2;
                   Wie*cos(Penu2_sins(1,i))+Venu_sins(1,i)*sec(Penu2_sins(1,i))^2/(Rn(i)+Penu2_sins(3,i)) 0 -Venu_sins(1,i)*tan(Penu2_sins(1,i))/(Rn(i)+Penu2_sins(3,i))^2];
    F32(:,:,i) = [ 0 -1/(Rm(i)+Penu2_sins(3,i)) 0;
                   1/(Rn(i)+Penu2_sins(3,i)) 0 0;
                   tan(Penu2_sins(1,i))/(Rn(i)+Penu2_sins(3,i)) 0 0];
    F33(:,:,i) = [ 0 Wie*sin(Penu2_sins(1,i))+Venu_sins(1,i)*tan(Penu2_sins(1,i))/(Rn(i)+Penu2_sins(3,i)) -(Wie*cos(Penu2_sins(1,i))+Venu_sins(1,i)/(Rn(i)+Penu2_sins(3,i)));
                  -(Wie*sin(Penu2_sins(1,i))+Venu_sins(1,i)*tan(Penu2_sins(1,i))/(Rn(i)+Penu2_sins(3,i))) 0 -Venu_sins(2,i)/(Rm(i)+Penu2_sins(3,i));
                   Wie*cos(Penu2_sins(1,i))+Venu_sins(1,i)/(Rn(i)+Penu2_sins(3,i)) Venu_sins(2,i)/(Rm(i)+Penu2_sins(3,i)) 0];
    F34(:,:,i) = zeros(3,5);
    F3(:,:,i) = horzcat(F31(:,:,i),F32(:,:,i),F33(:,:,i),-Cnb_sins(:,:,i)',F34(:,:,i));
    F4(:,:,i) = zeros(6,17);
    F5(:,:,i) = [zeros(1,16) 1;zeros(1,16) -1/1800];

    Fkf(:,:,i) = vertcat(F1(:,:,i),F2(:,:,i),F3(:,:,i),F4(:,:,i),F5(:,:,i)); 
    Fc = Fkf(:,:,i);

% ‘Î…˘æÿ’Û
    G1(:,:,i) = horzcat(zeros(3,3),Cnb_sins(:,:,i)',zeros(3,2));
    G2(:,:,i) = horzcat(Cnb_sins(:,:,i)',zeros(3,5));
    G3 = horzcat(zeros(6,6),zeros(6,2));
    G4(:,:,i) = [zeros(1,6) 1 0;zeros(1,7) 1];
    Gkf(:,:,i) = vertcat(zeros(3,8),G1(:,:,i),G2(:,:,i),G3,G4(:,:,i));
    Gc = Gkf(:,:,i);
 
% ¿Î…¢ªØ
    [Fd,Gd] = c2d(Fc,Gc,tstep);
    
% ¡ø≤‚æÿ’Û
    HPseudo11(1:4,1:3,i) = [(Pos_ecef_real(1,i)-GNSS1Pos(i,1))/Pseudoreal(1,i) (Pos_ecef_real(2,i)-GNSS1Pos(i,2))/Pseudoreal(1,i) (Pos_ecef_real(3,i)-GNSS1Pos(i,3))/Pseudoreal(1,i);
                        (Pos_ecef_real(1,i)-GNSS2Pos(i,1))/Pseudoreal(2,i) (Pos_ecef_real(2,i)-GNSS2Pos(i,2))/Pseudoreal(2,i) (Pos_ecef_real(3,i)-GNSS2Pos(i,3))/Pseudoreal(2,i);
                        (Pos_ecef_real(1,i)-GNSS3Pos(i,1))/Pseudoreal(3,i) (Pos_ecef_real(2,i)-GNSS3Pos(i,2))/Pseudoreal(3,i) (Pos_ecef_real(3,i)-GNSS3Pos(i,3))/Pseudoreal(3,i);
                        (Pos_ecef_real(1,i)-GNSS4Pos(i,1))/Pseudoreal(4,i) (Pos_ecef_real(2,i)-GNSS4Pos(i,2))/Pseudoreal(4,i) (Pos_ecef_real(3,i)-GNSS4Pos(i,3))/Pseudoreal(4,i)];
    HPseudo12(:,:,i) = [-(RmRn(2,i)+Pos_n2_real(3,i))*sin(Pos_n2_real(1,i))*cos(Pos_n2_real(2,i)) -(RmRn(2,i)+Pos_n2_real(3,i))*cos(Pos_n2_real(1,i))*sin(Pos_n2_real(2,i)) cos(Pos_n2_real(1,i))*cos(Pos_n2_real(2,i));
                        -(RmRn(2,i)+Pos_n2_real(3,i))*sin(Pos_n2_real(1,i))*sin(Pos_n2_real(2,i)) (RmRn(2,i)+Pos_n2_real(3,i))*cos(Pos_n2_real(1,i))*cos(Pos_n2_real(2,i)) cos(Pos_n2_real(1,i))*sin(Pos_n2_real(2,i));
                         (RmRn(2,i)*(1-e^2)+Pos_n2_real(3,i))*cos(Pos_n2_real(1,i)) 0 sin(Pos_n2_real(1,i))];
    HPseudo1(1:4,1:3,i) = HPseudo11(:,:,i)*HPseudo12(:,:,i);
    HPseudo2 = zeros(4,12);              
    HPseudo3 = [1 0;1 0;1 0;1 0];               
    HPseudo(:,:,i)= horzcat(HPseudo1(:,:,i),HPseudo2,HPseudo3);

    HPseudoRate111(:,:,i) = [-Vel_n_real(2,i)*cos(Pos_n2_real(1,i))*cos(Pos_n2_real(2,i))-Vel_n_real(3,i)*sin(Pos_n2_real(1,i))*cos(Pos_n2_real(2,i)) -Vel_n_real(1,i)*cos(Pos_n2_real(2,i))+Vel_n_real(2,i)*sin(Pos_n2_real(1,i))*sin(Pos_n2_real(2,i))-Vel_n_real(3,i)*cos(Pos_n2_real(1,i))*sin(Pos_n2_real(2,i)) 0
                             -Vel_n_real(2,i)*cos(Pos_n2_real(1,i))*sin(Pos_n2_real(2,i))-Vel_n_real(3,i)*sin(Pos_n2_real(1,i))*sin(Pos_n2_real(2,i)) -Vel_n_real(1,i)*sin(Pos_n2_real(2,i))-Vel_n_real(2,i)*sin(Pos_n2_real(1,i))*cos(Pos_n2_real(2,i))+Vel_n_real(3,i)*cos(Pos_n2_real(1,i))*cos(Pos_n2_real(2,i)) 0
                             -Vel_n_real(2,i)*sin(Pos_n2_real(1,i))+Vel_n_real(3,i)*cos(Pos_n2_real(1,i)) 0 0];
    HPseudoRate11211(:,i) = (Vel_ecef_real(1,i)-GNSS1V(i,1)-PseudoRatereal(1,i)*HPseudo11(1,1,i))/Pseudoreal(1,i);
    HPseudoRate11212(:,i) = (Vel_ecef_real(2,i)-GNSS1V(i,2)-PseudoRatereal(1,i)*HPseudo11(1,2,i))/Pseudoreal(1,i);
    HPseudoRate11213(:,i) = (Vel_ecef_real(3,i)-GNSS1V(i,3)-PseudoRatereal(1,i)*HPseudo11(1,3,i))/Pseudoreal(1,i);
    HPseudoRate11221(:,i) = (Vel_ecef_real(1,i)-GNSS2V(i,1)-PseudoRatereal(2,i)*HPseudo11(2,1,i))/Pseudoreal(2,i);
    HPseudoRate11222(:,i) = (Vel_ecef_real(2,i)-GNSS2V(i,2)-PseudoRatereal(2,i)*HPseudo11(2,2,i))/Pseudoreal(2,i);
    HPseudoRate11223(:,i) = (Vel_ecef_real(3,i)-GNSS2V(i,3)-PseudoRatereal(2,i)*HPseudo11(2,3,i))/Pseudoreal(2,i);
    HPseudoRate11231(:,i) = (Vel_ecef_real(1,i)-GNSS3V(i,1)-PseudoRatereal(3,i)*HPseudo11(3,1,i))/Pseudoreal(3,i);
    HPseudoRate11232(:,i) = (Vel_ecef_real(2,i)-GNSS3V(i,2)-PseudoRatereal(3,i)*HPseudo11(3,2,i))/Pseudoreal(3,i);
    HPseudoRate11233(:,i) = (Vel_ecef_real(3,i)-GNSS3V(i,3)-PseudoRatereal(3,i)*HPseudo11(3,3,i))/Pseudoreal(3,i);
    HPseudoRate11241(:,i) = (Vel_ecef_real(1,i)-GNSS4V(i,1)-PseudoRatereal(4,i)*HPseudo11(4,1,i))/Pseudoreal(4,i);
    HPseudoRate11242(:,i) = (Vel_ecef_real(2,i)-GNSS4V(i,2)-PseudoRatereal(4,i)*HPseudo11(4,2,i))/Pseudoreal(4,i);
    HPseudoRate11243(:,i) = (Vel_ecef_real(3,i)-GNSS4V(i,3)-PseudoRatereal(4,i)*HPseudo11(4,3,i))/Pseudoreal(4,i);   
    HPseudoRate112(:,:,i) = [ HPseudoRate11211(:,i) HPseudoRate11212(:,i) HPseudoRate11213(:,i);
                              HPseudoRate11221(:,i) HPseudoRate11222(:,i) HPseudoRate11223(:,i);
                              HPseudoRate11231(:,i) HPseudoRate11232(:,i) HPseudoRate11233(:,i);
                              HPseudoRate11241(:,i) HPseudoRate11242(:,i) HPseudoRate11243(:,i)];
    HPseudoRate11(:,:,i) = HPseudo11(:,:,i)*HPseudoRate111(:,:,i) + HPseudoRate112(:,:,i)*HPseudo12(:,:,i);
    HPseudoRate12(:,:,i) = HPseudo11(:,:,i)*Cne_real(:,:,i);
    HPseudoRate1(:,:,i) = horzcat(HPseudoRate11(:,:,i),HPseudoRate12(:,:,i));
    HPseudoRate2 = zeros(4,9);              
    HPseudoRate3 = [0 1;0 1;0 1;0 1]; 
    HPseudoRate(:,:,i)= horzcat(HPseudoRate1(:,:,i),HPseudoRate2,HPseudoRate3);

    HPkf(:,:,i) = vertcat(HPseudo(:,:,i),HPseudoRate(:,:,i));
    Hd = HPkf(:,:,i);
    
% ¡ø≤‚‘Î…˘
    Rkf(:,:,i)  = diag(RKF);
    Rd = Rkf(:,:,i);
    
% KF¬À≤®      
    if i==1
       Pkf(:,:,1) = diag(PKF); 
       Xkf(:,1) = zeros(17,1);
       PKFKF(:,1) = sqrt(diag(Pkf(:,:,1)));
    else
       Pkfk = Fd*Pkf(:,:,i-1)*Fd' + Gd*QKF*Gd';
       Kkf = Pkfk*Hd'*(inv(Hd*Pkfk*Hd'+Rd));
       Pkf(:,:,i) = (eye(17)-Kkf*Hd)*Pkfk;
       Xkf(:,i) = Fd*Xkf(:,i-1) + Kkf*(Z-Hd*(Fd*Xkf(:,i-1))); 
    end
    XKF(1:17,i) = Xkf(1:17,i); 
    PKFKF(1:17,i) = sqrt(diag(Pkf(:,:,i)));
    
%% ◊È∫œ∏¸–¬    
    Penu_gpsins(1,i) = Penu2_sins(1,i) - Xkf(1,i);
    Penu_gpsins(2,i) = Penu2_sins(2,i) - Xkf(2,i);
    Penu_gpsins(3,i) = Penu2_sins(3,i) - Xkf(3,i);
    Venu_gpsins(1,i) = Venu_sins(1,i) - Xkf(4,i);
    Venu_gpsins(2,i) = Venu_sins(2,i) - Xkf(5,i);
    Venu_gpsins(3,i) = Venu_sins(3,i) - Xkf(6,i);
    Cbn_Old = Cnb_sins(:,:,i)';
    Cbn_Update = [1 -Xkf(9,i) Xkf(8,i);
                   Xkf(9,i) 1 -Xkf(7,i);
                   -Xkf(8,i) Xkf(7,i) 1]*Cbn_Old;
    Cnb_Update = Cbn_Update';
    Attitude_gpsins(:,i) = genAttitudeSins(Cnb_Update);
    
    deltaEvaluBG(:,i) = Xkf(10:12,i);
    deltaEvaluBA(:,i) = Xkf(13:15,i);
    deltaEvaluBTu(i,:) =  Xkf(16,i);
    deltaEvaluBTru(i,:) =  Xkf(17,i); 
    
    Penu2_sins(1,i) = Penu_gpsins(1,i);
    Penu2_sins(2,i) = Penu_gpsins(2,i);
    Penu2_sins(3,i) = Penu_gpsins(3,i);
    Venu_sins(1,i) = Venu_gpsins(1,i);
    Venu_sins(2,i) = Venu_gpsins(2,i);
    Venu_sins(3,i) = Venu_gpsins(3,i);
    Attitude_sins(:,i) = Attitude_gpsins(:,i); 
    
 %% ∏≥÷µ◊º±∏±£¥Ê
    Penu_gpsinsPPR4(:,i) = Penu_gpsins(:,i);
    Venu_gpsinsPPR4(:,i) = Venu_gpsins(:,i);
    Attitude_gpsinsPPR4(:,i) = Attitude_gpsins(:,i);
    
    XKFPPR4(1:17,i) = XKF(1:17,i); 
    PKFKFPPR4(1:17,i) = PKFKF(1:17,i) ;    
end
%--------------------------------------------------------------------------
save('Penu_gpsinsPPR4');
save('Venu_gpsinsPPR4');
save('Attitude_gpsinsPPR4');
save('XKFPPR4');
save('PKFKFPPR4');