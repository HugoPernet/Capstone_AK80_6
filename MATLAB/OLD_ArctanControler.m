clear all;

x = linspace(-3,3,1000);
imu = linspace(3,-3,1000);

As = 2;
Al = 2;
Ss = 1;

KdMax = 3.0;
kdMin = 0.5;

zero_leg = -4.0;
zero_shoulder = 0.07;

Slack = abs(zero_shoulder-zero_leg);


shoulder = As*(1/pi)*atan(rad2deg(x-(Slack/2))-10) + As/2 ;
leg = Al*(1/pi)*atan(rad2deg(-x -(Slack/2)) +10)- As/2 - 2*Ss ;
switching = -5*(1/pi)*atan(rad2deg(imu)-20)+1;

KD = ((KdMax-kdMin)/2)*cos((2*pi/(Slack))*(x +(Slack/2))) + kdMin + ((KdMax-kdMin)/2);

KD2 = KdMax*(1/pi)*atan(rad2deg(x-(Slack/2))-10) + KdMax*(1/pi)*atan(rad2deg(-x -(Slack/2)) +10) +(KdMax +kdMin);

s1 = (1/pi)*atan(rad2deg(x - zero_shoulder)-10) + pi/2;
s2 = (1/pi)*atan(-rad2deg(imu)-10)+ 1/2;

tot = shoulder + leg+ switching;


subplot(3,2,1)
plot(rad2deg(x),shoulder,LineWidth=3)
title("shoulder",fontsize=15)
xlabel("$\theta_{mot} (degrees)$",Interpreter="latex",fontsize=15)
xregion(-100,100,"FaceColor", 'g'); % Note, 80 is the end point so if you pan past 80, the region will stop.
xregion(100, 200, "FaceColor", [1 0 0]);
xregion(-100, -200, "FaceColor", [0 0 0.6]); 
ylabel("$\tau_{mot} (N.m)$",Interpreter="latex",FontSize=15)
txt1 = {'Transition'};
text(-20,1,txt1, fontsize=15)
txt2 = {'Support','shoulder'};
text(140,1,txt2,fontsize=15)



subplot(3,2,2)
plot(rad2deg(x),leg,LineWidth=3)
title("leg",fontsize=15)
xlabel("$\theta_{mot} (degrees)$",Interpreter="latex",FontSize=15)
xregion(-100,100,"FaceColor", 'g'); % Note, 80 is the end point so if you pan pasx_solt 80, the region will stop.
xregion(100, 200, "FaceColor", [1 0 0]);
xregion(-100, -200, "FaceColor", [0 0 0.6]);
ylabel("$\tau_{mot} (N.m)$",Interpreter="latex",FontSize=15)
txt1 = {'Support leg '};
text(-170,-3,txt1, fontsize=15)
txt2 = {'Transistion'};
text(-20,-3,txt2,fontsize=15)


subplot(3,2,3)
plot(rad2deg(imu),switching,LineWidth=3)
title("imu",FontSize=15)
xlabel("$\theta_{imu} (degrees)$",Interpreter="latex",FontSize=15)
ylabel("$\tau_{mot} (N.m)$",Interpreter="latex",FontSize=15)

xregion(-100,100,"FaceColor", 'g');
xline(10)

subplot(3,2,4)
plot(rad2deg(x),tot);
title("total")
xlabel("$\theta_{mot} (degrees)$",Interpreter="latex")
xline(rad2deg(+zero_shoulder))
xline(rad2deg(zero_leg))
xline(-10)

subplot(3,2,5)
plot(imu,s2);
title("theta_dot")
xlabel("$\theta_{mot} (degrees)$",Interpreter="latex")
xline(10)
saveas(gcf,'ArctanController.jpeg')


