%% PlotGPSinsPPR4

load('Time');
load('Pos_n2_real');    % [Lat(rad), Lon(rad), Hei(m)]
load('Penu_gpsinsPPR4');
load('Venu_gpsinsPPR4');
load('Attitude_gpsinsPPR4');
load('XKFPPR4');
load('PKFKFPPR4');
%--------------------------------------------------------------------------

d2r = pi/180;
plotstep=1;
EndTime=Time(1);
t=1:plotstep:EndTime;

figIndex = 1;

figure(figIndex)
subplot(2,1,1)
plot(XKFPPR4(1,t)/d2r,'-b','LineWidth',2);hold on;
ylabel('Latitude(deg)');grid on;title('Latitude Error');
subplot(2,1,2)
plot(XKFPPR4(2,t)/d2r,'-b','LineWidth',2);hold on;
xlabel('time(s)');ylabel('Longitude(deg)');grid on;title('Longtitude Error');
figIndex = figIndex+1;

figure(figIndex)
plot(XKFPPR4(3,t),'-b','LineWidth',2);hold on;
xlabel('time(s)');ylabel('Height(m)');grid on;title('Height Error');
figIndex = figIndex+1;

figure(figIndex)
plot(XKFPPR4(4,t),'-b','LineWidth',2);hold on;
plot(XKFPPR4(5,t),'-g','LineWidth',2);hold on;
plot(XKFPPR4(6,t),'-m','LineWidth',2);hold on;
xlabel('time(s)');ylabel('Velocity error(m/s)');grid on;title('Velocity error');legend('Ve','Vn','Vu');
figIndex = figIndex+1;

figure(figIndex)
plot(XKFPPR4(7,t)/d2r,'-b','LineWidth',2);hold on;
plot(XKFPPR4(8,t)/d2r,'-g','LineWidth',2);hold on;
plot(XKFPPR4(9,t)/d2r,'-m','LineWidth',2);hold on;
xlabel('time(s)');ylabel('Attitude error(deg)');grid on;title('Attitude error');legend('Pitch','Roll','Yaw');
figIndex = figIndex+1;

figure(figIndex)
plot(XKFPPR4(10,t)/d2r*3600,'-b','LineWidth',2);hold on;
plot(XKFPPR4(11,t)/d2r*3600,'-g','LineWidth',2);hold on;
plot(XKFPPR4(12,t)/d2r*3600,'-m','LineWidth',2);hold on;
xlabel('time(s)');ylabel('Gyro Bias error(deg/h)');grid on;title('Gyro Bias error');legend('b_G_M_E','b_G_M_N','b_G_M_U');
figIndex = figIndex+1;

figure(figIndex)
plot(XKFPPR4(13,t)/(1e-3*g),'-b','LineWidth',2);hold on;
plot(XKFPPR4(14,t)/(1e-3*g),'-g','LineWidth',2);hold on;
plot(XKFPPR4(15,t)/(1e-3*g),'-m','LineWidth',2);hold on;
xlabel('time(s)');ylabel('Accel Bias error(mg)');grid on;title('Accel Bias error');legend('b_A_M_E','b_A_M_N','b_A_M_U');
figIndex = figIndex+1;

figure(figIndex)
subplot(2,1,1)
plot(XKFPPR4(16,t),'-b','LineWidth',2);hold on;
ylabel('Clock Bias error(m)');grid on;title('Clock Bias error');
subplot(2,1,2)
plot(XKFPPR4(17,t),'-b','LineWidth',2);hold on;
xlabel('time(s)');ylabel('Clock drift error(m/s)');grid on;title('Clock drift error');
figIndex = figIndex+1;

figure(figIndex)
subplot(2,1,1)
plot(PKFKFPPR4(1,t)/d2r,'-b','LineWidth',2);hold on;
ylabel('Latitude Variance(deg)');grid on;title('Latitude Error');
subplot(2,1,2)
plot(PKFKFPPR4(2,t)/d2r,'-b','LineWidth',2);hold on;
xlabel('time(s)');ylabel('Longitude Variance(deg)');grid on;title('Longtitude Error');
figIndex = figIndex+1;

figure(figIndex)
plot(PKFKFPPR4(3,t),'-b','LineWidth',2);hold on;
xlabel('time(s)');ylabel('Height Variance(m)');grid on;title('Height Error');
figIndex = figIndex+1;

figure(figIndex)
plot(PKFKFPPR4(4,t),'-b','LineWidth',2);hold on;
plot(PKFKFPPR4(5,t),'-g','LineWidth',2);hold on;
plot(PKFKFPPR4(6,t),'-m','LineWidth',2);hold on;
xlabel('time(s)');ylabel('Velocity error Variance(m/s)');grid on;title('Velocity error');legend('Ve','Vn','Vu');
figIndex = figIndex+1;

figure(figIndex)
plot(PKFKFPPR4(7,t)/d2r,'-b','LineWidth',2);hold on;
plot(PKFKFPPR4(8,t)/d2r,'-g','LineWidth',2);hold on;
plot(PKFKFPPR4(9,t)/d2r,'-m','LineWidth',2);hold on;
xlabel('time(s)');ylabel('Attitude error Variance(deg)');grid on;title('Attitude error');legend('Pitch','Roll','Yaw');
figIndex = figIndex+1;

figure(figIndex)
plot(PKFKFPPR4(10,t)/d2r*3600,'-b','LineWidth',2);hold on;
plot(PKFKFPPR4(11,t)/d2r*3600,'-g','LineWidth',2);hold on;
plot(PKFKFPPR4(12,t)/d2r*3600,'-m','LineWidth',2);hold on;
xlabel('time(s)');ylabel('Gyro Bias error Variance(deg/h)');grid on;title('Gyro Bias error');legend('x','y','z');
figIndex = figIndex+1;

figure(figIndex)
plot(PKFKFPPR4(13,t)/(1e-3*g),'-b','LineWidth',2);hold on;
plot(PKFKFPPR4(14,t)/(1e-3*g),'-g','LineWidth',2);hold on;
plot(PKFKFPPR4(15,t)/(1e-3*g),'-m','LineWidth',2);hold on;
xlabel('time(s)');ylabel('Accel Bias error Variance(mg)');grid on;title('Accel Bias error');legend('x','y','z');
figIndex = figIndex+1;

figure(figIndex)
subplot(211)
plot(PKFKFPPR4(16,t),'-b','LineWidth',2);hold on;
ylabel('Clock Bias error Variance(m)');grid on;title('Clock Bias error');
subplot(2,1,2)
plot(PKFKFPPR4(17,t),'-m','LineWidth',2);hold on;
xlabel('time(s)');ylabel('Clock drift error Variance(m/s)');grid on;title('Clock drift error');

