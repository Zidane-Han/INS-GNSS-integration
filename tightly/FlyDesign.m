%% FlyDesign 

clear all
close all

%--------------------------------------------------------------------------
%% 导航坐标系ENU下飞行轨迹的设计
t0=3600;           %空中飞行总时间;
Time=[t0];
d2r=pi/180;
initvel=1;        %飞行速度（单位：马赫）
initPos = [30*d2r;120*d2r;5000];


for i=1:1:t0      %全过程
    btheta(i)=0;  %弹体俯仰角（弹体坐标与地面坐标的角关系）  
    gama(i)=0;    %弹体横滚角
    fai(i)=0;     %弹体航向角
    theta(i)=0;   %弹道倾角（弹道坐标与地面坐标的角关系）
    faiv(i)=0;    %弹道偏角
    beta(i)=0;    %弹体测滑角（弹体坐标与速度坐标的角关系）
    arfa(i)=0;    %弹体攻角
    gamav(i)=0;   %弹道速度倾斜角（弹道坐标与速度坐标的角关系）   
    
    ma(i)=initvel; 
end

for i=1:1:t0 
    
    AngleDesign(1,i)=btheta(i)*d2r;  % rad  
    AngleDesign(2,i)=gama(i)*d2r;        
    AngleDesign(3,i)=fai(i)*d2r;
    AngleDesign(4,i)=theta(i)*d2r;
    AngleDesign(5,i)=faiv(i)*d2r;
    AngleDesign(6,i)=beta(i)*d2r;    
    AngleDesign(7,i)=arfa(i)*d2r;
    AngleDesign(8,i)=gamav(i)*d2r;   
    
    VelocityDesign(1,i)=ma(i)*340;    % m/s
end
%--------------------------------------------------------------------------
%% 保存弹道轨迹数据
save('Time');
save('AngleDesign');
save('VelocityDesign');
save('initPos');
