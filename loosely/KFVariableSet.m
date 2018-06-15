% KFVariableSet
% P_k初值设定：accurancy of state parameter
% P_k(1,1) P_k(2,2) P_k(3,3) attitude^2
% P_k(4,4) P_k(5,5) P_k(6,6) velocity^2
% P_k(7,7) P_k(8,8) P_k(9,9) position^2
% P_k(10,10) P_k(11,11) P_k(12,12) P_k(13,13) P_k(14,14) P_k(15,15)  errors^2 by gyro
% P_k(16,16) P_k(17,17) P_k(18,18) errors^2 by accelerator

% VQg,VQr,VQa   状态变量噪声方差阵
% VR  系统噪声序列by GPS
     
     Xt=zeros(15,1);                  X_k=zeros(15,1);                 Xk(:,1)=X_k;                     
     Ft=zeros(15,15);                 Fdk=zeros(15,15);
     Gt=zeros(15,9);                  Gdk=zeros(15,9); 
     
     Fins=zeros(9,9);                 T=zeros(9,9);  
     P_k=zeros(15);                   P_kk1=zeros(15);                  Pk_diag=zeros(15,1);    
     number=1;

     % P_k初值设定
     P_k(1,1)=0^2;                    P_k(2,2)=0^2;                     P_k(3,3)=0^2;
     P_k(4,4)=0^2;                    P_k(5,5)=0^2;                     P_k(6,6)=0^2;
     P_k(7,7)=0^2;                    P_k(8,8)=0^2;                     P_k(9,9)=0^2;
     P_k(10,10)=(1/3600*d2r)^2;       P_k(11,11)=(1/3600*d2r)^2;        P_k(12,12)=(1/3600*d2r)^2;
     P_k(13,13)=(0.001*g)^2;          P_k(14,14)=(0.001*g)^2;           P_k(15,15)=(0.001*g)^2; 
     Pk_Diag(:,1)=diag(P_k);
     %VQ&VR 功率谱密度
     VQg_noise=wgn(3,1,10*log10(0.00001));
     VQa_noise=wgn(3,1,10*log10(0.00001));
     VQgx=(VQg_noise(1))^2;  VQgy=(VQg_noise(2))^2;  VQgz=(VQg_noise(3))^2;  
     VQax=(VQa_noise(1))^2;  VQay=(VQa_noise(2))^2;  VQaz=(VQa_noise(3))^2;  
     VQ=diag([VQgx,VQgy,VQgz,VQax,VQay,VQaz]);
     
     VRp_noise(1:2,1)=wgn(2,1,10*log10(10e-20));
     VRp_noise(3,1)=wgn(1,1,10*log10(5));
     VRv_noise=wgn(3,1,10*log10(0.001));
     VRpx=(0.5e-7)^2;  VRpy=(0.5e-7)^2;  VRpz=(1)^2; 
     VRvx=(0.05)^2;  VRvy=(0.05)^2;  VRvz=(0.05)^2;  
     VR=diag([VRpx,VRpy,VRpz,VRvx,VRvy,VRvz]);                    