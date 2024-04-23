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

midpoint = 2.04;
As = 3; Al = 2; Ss = 1; L1 = 4.5; L2 = 2.5;
shoulder = (1/pi)*atan(rad2deg(x-midpoint)-10) +0.5;
leg = (1/pi)*atan(-rad2deg(x+midpoint) -3)+0.5; 


switching = (1/pi)*atan(rad2deg(imu)-20)+0.5 ;

Ts = As*sin(rad2deg(x-midpoint)/10) + As;
Tleg = As*sin(rad2deg(x+midpoint)/10) -As

Ts_switch = (As*sin(rad2deg(x-midpoint)/10) + As).*(1-((1/pi)*atan(rad2deg(imu)-20)+0.5));
Tleg_switch = - As*sin(rad2deg(x+midpoint)/10).*((1/pi)*atan(rad2deg(imu)-20)+0.5)


tot = shoulder + leg+ switching;



subplot(3,2,1)
plot(rad2deg(imu),switching,LineWidth=3)
title("imu",FontSize=15)
xlabel("$\theta_{imu} (degrees)$",Interpreter="latex",FontSize=15)
ylabel("$\tau_{mot} (N.m)$",Interpreter="latex",FontSize=15)
xregion(10,200,"FaceColor", 'g');
xregion(-200,10,"FaceColor", 'R');

txt1 = {'standing up'};
text(-100,0.5,txt1, fontsize=15)
txt2 = {'bending'};
text(100,0.5,5,txt2,fontsize=15)



subplot(3,2,2)
plot(rad2deg(x),Ts,LineWidth=3)
title("T shoulder",FontSize=15)
xlabel("$\theta_{imu} (degrees)$",Interpreter="latex",FontSize=15)
ylabel("$\tau_{mot} (N.m)$",Interpreter="latex",FontSize=15)
xregion(-100,100,"FaceColor", 'g'); % Note, 80 is the end point so if you pan past 80, the region will stop.
xregion(100, 200, "FaceColor", [1 0 0]);
xregion(-100, -200, "FaceColor", [0 0 0.6]); 
ylabel("$\tau_{mot} (N.m)$",Interpreter="latex",FontSize=15)
txt1 = {'leg region'};
text(-180,3,txt1, fontsize=15)
txt2 = {'slack'};
text(-20,3,5,txt2,fontsize=15)
txt3 = {'Shoulder'};
text(-20,3,5,txt2,fontsize=15)



subplot(3,2,3)
plot(rad2deg(x),Tleg,LineWidth=3)
title("T leg",FontSize=15)
xlabel("$\theta_{imu} (degrees)$",Interpreter="latex",FontSize=15)
ylabel("$\tau_{mot} (N.m)$",Interpreter="latex",FontSize=15)
xregion(-100,100,"FaceColor", 'g'); % Note, 80 is the end point so if you pan past 80, the region will stop.
xregion(100, 200, "FaceColor", [1 0 0]);
xregion(-100, -200, "FaceColor", [0 0 0.6]); 
ylabel("$\tau_{mot} (N.m)$",Interpreter="latex",FontSize=15)

subplot(3,2,4)
plot(rad2deg(x),Tleg_switch,LineWidth=3)
title("T leg switched",FontSize=15)
xlabel("$\theta_{imu} (degrees)$",Interpreter="latex",FontSize=15)
ylabel("$\tau_{mot} (N.m)$",Interpreter="latex",FontSize=15)
xregion(-100,100,"FaceColor", 'g'); % Note, 80 is the end point so if you pan past 80, the region will stop.
xregion(100, 200, "FaceColor", [1 0 0]);
xregion(-100, -200, "FaceColor", [0 0 0.6]); 
ylabel("$\tau_{mot} (N.m)$",Interpreter="latex",FontSize=15)

subplot(3,2,5)
plot(rad2deg(x),Ts_switch,LineWidth=3)
title("T shoulder switched",FontSize=15)
xlabel("$\theta_{imu} (degrees)$",Interpreter="latex",FontSize=15)
ylabel("$\tau_{mot} (N.m)$",Interpreter="latex",FontSize=15)
xregion(-100,100,"FaceColor", 'g'); % Note, 80 is the end point so if you pan past 80, the region will stop.
xregion(100, 200, "FaceColor", [1 0 0]);
xregion(-100, -200, "FaceColor", [0 0 0.6]); 
ylabel("$\tau_{mot} (N.m)$",Interpreter="latex",FontSize=15)

subplot(3,2,6)
plot(rad2deg(x),Ts_switch + Tleg_switch,LineWidth=3)
title("T mot",FontSize=15)
xlabel("$\theta_{imu} (degrees)$",Interpreter="latex",FontSize=15)
ylabel("$\tau_{mot} (N.m)$",Interpreter="latex",FontSize=15)
xregion(-100,100,"FaceColor", 'g'); % Note, 80 is the end point so if you pan past 80, the region will stop.
xregion(100, 200, "FaceColor", [1 0 0]);
xregion(-100, -200, "FaceColor", [0 0 0.6]); 
ylabel("$\tau_{mot} (N.m)$",Interpreter="latex",FontSize=15)


