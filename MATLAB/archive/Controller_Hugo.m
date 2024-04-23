clear, clc, close all

x = linspace(-3,3,1000);
imu = linspace(3,-3,1000);

As = 2;
Al = 4;
Ss = 0.5;

midpoint = 2.04;


shoulder = As*(1/pi)*atan(rad2deg(x-midpoint)-10) + As/2 + Ss;
leg = Al*(1/pi)*atan(rad2deg(x+midpoint) +20)- As/2 - Ss;
switching = -2*Ss*(1/pi)*atan(rad2deg(imu)-20) ;

% As = 2;
% Al = 1;
% Ss = 1;
% 
% shoulder = As*(1/pi)*atan(rad2deg(x-midpoint)-10) + As/2;
% leg = Al*(1/pi)*atan(rad2deg(x+midpoint) +10)- As/2 - 2*Ss+1;
% switching = -2*Ss*(1/pi)*atan(rad2deg(imu)-20)+1 ;

lowerbound_kd = 0.2;
Ad = 2*lowerbound_kd;
%kd = Ad*cos(pi/rad2deg(midpoint)*rad2deg(x-midpoint))+(1-Ad);
% kd = Ad*cos(pi/midpoint * (x-midpoint))+(1-Ad);
kd = Ad*cos((pi/midpoint)*(x-midpoint))+(1-Ad);

tot = shoulder + leg+ switching;


subplot(3,2,1)
plot(rad2deg(x),shoulder)
title("shoulder")
xlabel("$\theta_{mot} (degrees)$",Interpreter="latex")
xline(rad2deg(+midpoint))

subplot(3,2,2)
plot(rad2deg(x),leg)
title("leg")
xlabel("$\theta_{mot} (degrees)$",Interpreter="latex")
xline(rad2deg(-midpoint))

subplot(3,2,3)
plot(rad2deg(imu),switching)
title("imu")
xlabel("$\theta_{imu} (degrees)$",Interpreter="latex")
xline(10)

subplot(3,2,4)
plot(rad2deg(x),tot);
title("total")
xlabel("$\theta_{mot} (degrees)$",Interpreter="latex")
xline(rad2deg(+midpoint))
xline(rad2deg(-midpoint))
xline(-10)

subplot(3,2,5)
plot(rad2deg(x),kd);
title("kd")
xlabel("$\theta_{mot} (degrees)$",Interpreter="latex")
xline(rad2deg(+midpoint))
xline(rad2deg(-midpoint))
xline(-10)



% subplot(3,2,6)
% plot(rad2deg(x),tot+kd);
% title("total")
% xlabel("$\theta_{mot} (degrees)$",Interpreter="latex")
% xline(rad2deg(+midpoint))
% xline(rad2deg(-midpoint))
% xline(-10)


