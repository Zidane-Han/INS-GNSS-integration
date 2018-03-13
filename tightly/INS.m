%% INS解算

load('Time');
load('Wibb');            % [rad;rad;rad]           
load('Fb');              % [m/s^2;m/s^2;m/s^2]
load('RmRn');            % [Rm(m);Rn(m)]
load('Pos_n1_real');     % [m;m;Hei(m)]
load('Pos_n2_real');     % [Lat(rad);Lon(rad);Hei(m)]
load('Vel_n_real');      % [m/s; m/s;m/s]
load('Attitude_real');   % [Roll(rad); Pitch(rad); Yaw(rad)]
load('Cnb_real');
load('Pos_ecef_real');   % [m; m;m]
load('Vel_ecef_real');   % [m/s; m/s;m/s]
%--------------------------------------------------------------------------
g = 9.8;
fg = [0;0;-g];
t = 1;

for i=1:Time(1) 
    
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
    else     
       [Rm(i),Rn(i),wien(:,i),wenn(:,i)] = genWienWenn(Penu2_sins(:,i-1),Venu_sins(:,i-1));% 姿态解算 
       wnbbsins(:,i) = Wibb(:,i) - Cnb_sins(:,:,i-1)*(wien(:,i)+wenn(:,i));    
       Attitude_sins(1,i) = cos(Attitude_sins(2,i-1))*wnbbsins(1,i) + sin(Attitude_sins(2,i-1))*wnbbsins(3,i) + Attitude_sins(1,i-1);
       Attitude_sins(2,i) = sin(Attitude_sins(2,i-1))*tan(Attitude_sins(1,i-1))*wnbbsins(1,i)+ wnbbsins(2,i) - cos(Attitude_sins(2,i-1))*tan(Attitude_sins(1,i-1))*wnbbsins(3,i) + Attitude_sins(2,i-1);
       Attitude_sins(3,i) = 1/cos(Attitude_sins(1,i-1))*(sin(Attitude_sins(2,i-1))*wnbbsins(1,i)-cos(Attitude_sins(2,i-1))*wnbbsins(3,i)) + Attitude_sins(3,i-1);
       Cnb_sins(:,:,i) = genCnbsins(Attitude_sins(:,i));
       
       CmpV(:,i) = genCmpV(wien(:,i),wenn(:,i),Venu_sins(:,i-1));                           % 速度解算
       Venu_sins(:,i) = Venu_sins(:,i-1) + Cnb_sins(:,:,i)'*Fb(:,i) - CmpV(:,i) + fg; 
       
       Penu2_sins(3,i) = Penu2_sins(3,i-1) + Venu_sins(3,i);                                % 位置解算
       Penu2_sins(1,i) = Penu2_sins(1,i-1) + Venu_sins(2,i)/(Rm(i) + Penu2_sins(3,i));                          
       Penu2_sins(2,i) = Penu2_sins(2,i-1) + Venu_sins(1,i)/((Rn(i) + Penu2_sins(3,i))*cos(Penu2_sins(1,i)));
       Penu1_sins(:,i) = Penu1_sins(:,i-1) + Venu_sins(:,i)*t;

       Pecef_sins(1,i) = (Rn(i) + Penu2_sins(3,i))*cos(Penu2_sins(1,i))*cos(Penu2_sins(2,i));
       Pecef_sins(2,i) = (Rn(i) + Penu2_sins(3,i))*cos(Penu2_sins(1,i))*sin(Penu2_sins(2,i));
       Pecef_sins(3,i) = (Rn(i)*(1-e^2) + Penu2_sins(3,i))*sin(Penu2_sins(1,i)); 
    end     
    
       Cne_sins(:,:,i) = [-sin(Penu2_sins(2,i)) -sin(Penu2_sins(1,i))*cos(Penu2_sins(2,i)) cos(Penu2_sins(1,i))*cos(Penu2_sins(2,i));
                           cos(Penu2_sins(2,i)) -sin(Penu2_sins(1,i))*sin(Penu2_sins(2,i)) cos(Penu2_sins(1,i))*sin(Penu2_sins(2,i));
                           0 cos(Penu2_sins(1,i)) sin(Penu2_sins(1,i))];
       Vecef_sins(:,i) = Cne_sins(:,:,i)*Venu_sins(:,i); 
end
%--------------------------------------------------------------------------
save('Penu1_sins');   % [Pe=Ve*t(m); Pn=Vn*t(m); Hei(m)]
save('Penu2_sins');   % [Lat(rad); Lon(rad); Hei(m)]
save('Venu_sins');    % [m/s; m/s; m/s]
save('Attitude_sins');% [Roll(rad); Pitch(rad); Yaw(rad)]
save('Pecef_sins');   % [m; m; m]
save('Vecef_sins');   % [m/s; m/s; m/s]
save('Cnb_sins');
save('Cne_sins');
