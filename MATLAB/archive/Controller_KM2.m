%% 
clear, clc, close all

x = linspace(-3,3,1000);
imu = linspace(3,-3,1000);

midpoint = 2.04;

As = 3; Al = 2; Ss = 1; L1 = 4.5; L2 = 2.5;
shoulder = As*(1/pi)*atan(rad2deg(x-midpoint)-3) + As/2+0.5;
leg = Al*(1/pi)*atan(-rad2deg(x+midpoint) +30)- As/2 - 2*Ss+0.5;
switching = -4*(1/pi)*atan(rad2deg(imu)-20)+2 ;
tot3 = shoulder + leg + switching;

As = 2; Al = 2; Ss = 1; L1 = 4.5; L2 = 2.5;
shoulder = As*(1/pi)*atan(rad2deg(x-midpoint)-3) + As/2+1;
leg = Al*(1/pi)*atan(-rad2deg(x+midpoint) +30)- As/2 - 2*Ss+0.5;
switching = -5*(1/pi)*atan(rad2deg(imu)-20)+1.5 ;
tot4 = shoulder+leg+switching;


Av1 = 1/pi;
motorang = Av1*atan(rad2deg(x-midpoint))+0.5;
Av2 = 1/pi;
motorvel = Av2*atan(-rad2deg(x)-10)+0.5;
Av3 = 1/pi;
imuvel = Av3*atan(rad2deg(x)-10)+0.5;

figure(1)
subplot(1,2,1)
plot(rad2deg(x), motorang); hold on
plot(rad2deg(x), motorvel); hold on
plot(rad2deg(x), imuvel); hold on
xline(rad2deg(midpoint)); xline(rad2deg(-midpoint))
legend('Motor Angle','Motor Velocity','IMU velocity','Location','NorthWest')

subplot(1,2,2)
plot(rad2deg(x),tot3)
title("total")
xlabel("$\theta_{mot} (degrees)$",Interpreter="latex")
xline(rad2deg(+midpoint)); xline(rad2deg(-midpoint))
xline(-10); yline(0) 
set(gcf,'color','w')


%% Plotting each component
figure(2)
subplot(2,2,1)
plot(rad2deg(x),shoulder)
title("shoulder")
xlabel("$\theta_{mot} (degrees)$",Interpreter="latex")
xline(rad2deg(+midpoint))

subplot(2,2,2)
plot(rad2deg(x),leg)
title("leg")
xlabel("$\theta_{mot} (degrees)$",Interpreter="latex")
xline(rad2deg(-midpoint))

subplot(2,2,3)
plot(rad2deg(imu),switching)
title("imu")
xlabel("$\theta_{imu} (degrees)$",Interpreter="latex")
xline(10)

subplot(2,2,4)
plot(rad2deg(x),tot3); hold on
plot(rad2deg(x),tot4)
title("total")
xlabel("$\theta_{mot} (degrees)$",Interpreter="latex")
xline(rad2deg(+midpoint))
xline(rad2deg(-midpoint))
xline(-10)
yline(0) 