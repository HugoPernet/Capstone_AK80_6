clear all
close all

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
Switch_shoulder = 1-(1/pi)*atan(rad2deg(x+midpoint)-10) -0.5;
Switch_leg = 1-(1/pi)*atan(-rad2deg(x-midpoint) -3)-0.5; 


switching = 1-(1/pi)*atan(rad2deg(imu)+20)-1 ;

Ts = As*sin(rad2deg(x-midpoint)/10) + As;
Tleg = As*sin(rad2deg(x+midpoint)/10) -As

Ts_switch = (As*sin(rad2deg(x-midpoint)/10) + As).*((1/pi)*atan(rad2deg(x-midpoint)+18) +0.5);
Tleg_switch = (As*sin(rad2deg(x+midpoint)/10) -As).*((1/pi)*atan(-rad2deg(x+midpoint) +20)+0.5)



figure(1)

subplot(4,2,1)
plot(rad2deg(imu),Switch_leg,LineWidth=3)
title("$SW_{leg}$ (Switching)",Interpreter="latex",FontSize=20)
xlabel("$\theta_{mot} (degrees)$",Interpreter="latex",FontSize=15)
ylabel("ON/OFF",Interpreter="latex",FontSize=15)
xregion(-100,200,"FaceColor", 'g');
xregion(-200,-100,"FaceColor", 'b');
txt1 = {'Leg ON'};
text(-180,0.5,txt1, fontsize=15)
txt2 = {'Leg OFF'};
text(50,0.5,5,txt2,fontsize=15)

subplot(4,2,2)
plot(rad2deg(imu),Switch_shoulder,LineWidth=3)
title("$SW_{shoulder}$ (Switching)",Interpreter="latex",FontSize=20)
xlabel("$\theta_{mot} (degrees)$",Interpreter="latex",FontSize=15)
ylabel("ON/OFF",Interpreter="latex",FontSize=15)
xregion(100,200,"FaceColor", 'r');
xregion(-200,100,"FaceColor", 'g');
txt1 = {'Shoulder OFF'};
text(-100,0.5,txt1, fontsize=15)
txt2 = {'Shoulder',' ON'};
text(120,0.5,5,txt2,fontsize=15)



subplot(4,2,3)
plot(rad2deg(x),Tleg,LineWidth=3)
title("$\tau_{leg} $",Interpreter="latex",FontSize=20)
xlabel("$\theta_{mot} (degrees)$",Interpreter="latex",FontSize=15)
ylabel("$\tau_{mot} (N.m)$",Interpreter="latex",FontSize=15)
xregion(-100,100,"FaceColor", 'g'); % Note, 80 is the end point so if you pan past 80, the region will stop.
xregion(100, 200, "FaceColor", [1 0 0]);
xregion(-100, -200, "FaceColor", [0 0 0.6]); 
ylabel("$\tau_{mot} (N.m)$",Interpreter="latex",FontSize=15)
txt1 = {'Leg','ON'};
text(-180,-3,txt1, fontsize=15)
txt2 = {'Slack'};
text(-20,-3,5,txt2,fontsize=15)
txt3 = {'Shoulder','ON'};
text(120,-3,5,txt3,fontsize=15)




subplot(4,2,4)
plot(rad2deg(x),Ts,LineWidth=3)
title("$\tau_{shoulder} $",Interpreter="latex",FontSize=20)
xlabel("$\theta_{mot} (degrees)$",Interpreter="latex",FontSize=15)
ylabel("$\tau_{mot} (N.m)$",Interpreter="latex",FontSize=15)
xregion(-100,100,"FaceColor", 'g'); % Note, 80 is the end point so if you pan past 80, the region will stop.
xregion(100, 200, "FaceColor", [1 0 0]);
xregion(-100, -200, "FaceColor", [0 0 0.6]); 
ylabel("$\tau_{mot} (N.m)$",Interpreter="latex",FontSize=15)
txt1 = {'Leg','ON'};
text(-180,3,txt1, fontsize=15)
txt2 = {'Slack'};
text(-20,3,5,txt2,fontsize=15)
txt3 = {'Shoulder','ON'};
text(120,3,5,txt3,fontsize=15)

subplot(4,2,5)
plot(rad2deg(x),Tleg_switch,LineWidth=3)
title("$\tau_{leg}^{SW}$",Interpreter="latex",FontSize=20)
xlabel("$\theta_{mot} (degrees)$",Interpreter="latex",FontSize=15)
ylabel("$\tau_{mot} (N.m)$",Interpreter="latex",FontSize=15)
xregion(-100,100,"FaceColor", 'g'); % Note, 80 is the end point so if you pan past 80, the region will stop.
xregion(100, 200, "FaceColor", [1 0 0]);
xregion(-100, -200, "FaceColor", [0 0 0.6]); 
ylabel("$\tau_{mot} (N.m)$",Interpreter="latex",FontSize=15)
txt1 = {'Leg','ON'};
text(-180,-3,txt1, fontsize=15)
txt2 = {'Slack'};
text(-20,-3,5,txt2,fontsize=15)
txt3 = {'Shoulder','ON'};
text(120,-3,5,txt3,fontsize=15)


subplot(4,2,6)
plot(rad2deg(x),Ts_switch,LineWidth=3)
title("$\tau_{shoulder}^{SW}$",Interpreter="latex",FontSize=20)
xlabel("$\theta_{mot} (degrees)$",Interpreter="latex",FontSize=15)
ylabel("$\tau_{mot} (N.m)$",Interpreter="latex",FontSize=15)
xregion(-100,100,"FaceColor", 'g'); % Note, 80 is the end point so if you pan past 80, the region will stop.
xregion(100, 200, "FaceColor", [1 0 0]);
xregion(-100, -200, "FaceColor", [0 0 0.6]); 
ylabel("$\tau_{mot} (N.m)$",Interpreter="latex",FontSize=15)
txt1 = {'Leg','ON'};
text(-180,3,txt1, fontsize=15)
txt2 = {'Slack'};
text(-20,3,5,txt2,fontsize=15)
txt3 = {'Shoulder','ON'};
text(120,3,5,txt3,fontsize=15)


subplot(4,2,7)
plot(rad2deg(x),Ts_switch + Tleg_switch,LineWidth=3)
title("$\tau_{motor} = \tau_{shoulder}^{SW} + \tau_{leg}^{SW}$",Interpreter="latex",FontSize=20)
xlabel("$\theta_{mot} (degrees)$",Interpreter="latex",FontSize=15)
ylabel("$\tau_{mot} (N.m)$",Interpreter="latex",FontSize=15)
xregion(-100,100,"FaceColor", 'g'); % Note, 80 is the end point so if you pan past 80, the region will stop.
xregion(100, 200, "FaceColor", [1 0 0]);
xregion(-100, -200, "FaceColor", [0 0 0.6]); 
ylabel("$\tau_{mot} (N.m)$",Interpreter="latex",FontSize=15)
txt1 = {'Leg','ON'};
text(-180,3,txt1, fontsize=15)
txt2 = {'Slack'};
text(-20,3,5,txt2,fontsize=15)
txt3 = {'Shoulder','ON'};
text(120,3,5,txt3,fontsize=15)

subplot(4,2,8)
plot(rad2deg(x),switching,LineWidth=3)
title("$SW_{imu}$", Interpreter="latex",fontsize=20)
xlabel("$\theta_{imu} (degrees)$",Interpreter="latex",FontSize=15)
ylabel("$\tau_{mot} (N.m)$",Interpreter="latex",FontSize=15)
xregion(10,200,"FaceColor", [0 0.7 0]); % Note, 80 is the end point so if you pan past 80, the region will stop.
xregion(-200, 10, "FaceColor", [0 1 0]);
ylabel("$\tau_{mot} (N.m)$",Interpreter="latex",FontSize=15)



%%%%%%%%%%%%%%%%%%%%% 3d graph %%%%%%%%%%%%%%%

%colormaps
mymap1 = [0 0 0.7
    0 0 0.7
    0 0 0.7
    0 0 0.7
    1 1 1
    1 1 1
    1 1 1
    1 1 1
    1 1 1
    1 1 1
    1 1 1
    1 1 1
    1 1 1
    1 1 1
    1 1 1
    0.7 0 0
    0.7 0 0
    0.7 0 0
    0.7 0 0];

mymap2 = [0 1 0
    0 0.5 0
    0 0.5 0];


%% plot surfaces
midpoint = 2.04;
As = 3; Al = 2; Ss = 1; L1 = 4.5; L2 = 2.5;

%create surface
[X,Y] = meshgrid(-3:0.1:3,-0.5:0.1:2.2); %some x and y data
Ts_switch = (As*sin(rad2deg(X-midpoint)/10) + As).*((1/pi)*atan(rad2deg(X-midpoint)+18) +0.5);
Tleg_switch = (As*sin(rad2deg(X+midpoint)/10) -As).*((1/pi)*atan(-rad2deg(X+midpoint) +20)+0.5)
switching = 1-(1/pi)*atan(rad2deg(Y)-20)-1 ;
Z = Ts_switch+Tleg_switch+switching;

%Second green surface to use a different color map on it
[X2,Y2] = meshgrid(-midpoint+0.25:0.1:midpoint-0.20,-0.5:0.1:2.2);
Ts_switch2 = (As*sin(rad2deg(X2-midpoint)/10) + As).*((1/pi)*atan(rad2deg(X2-midpoint)+18) +0.5);
Tleg_switch2 = (As*sin(rad2deg(X2+midpoint)/10) -As).*((1/pi)*atan(-rad2deg(X2+midpoint) +20)+0.5)
switching2 = 1-(1/pi)*atan(rad2deg(Y2)-20)-1 ;
Z2 = Ts_switch2+Tleg_switch2+switching2;

figure(2)
ax1 = axes;
title("$\tau_{motor} = \tau_{shoulder}^{SW} + \tau_{leg}^{SW} + SW_{imu}$",Interpreter="latex",FontSize=25)
surf(rad2deg(X),rad2deg(Y),Z,X,'FaceAlpha',0.3,'EdgeAlpha',0.3,'DisplayName', 'torque'); %surface plot
ax2 = axes;
surf(rad2deg(X2),rad2deg(Y2),Z2,Y2,'FaceAlpha',0.6,'EdgeAlpha',0.3);
ax2.Visible = 'off';

% sketchy stuff to superpose the two surfaces correctly
hLink = linkprop([ax1,ax2],{'xLim','yLim','zLim','CameraUpVector','CameraPosition','CameraTarget'});
% Give each one its colormap
colormap(ax1,mymap1)
colormap(ax2,mymap2)
setappdata(gcf,'StoreTheLink',hLink);
% Add colorbars and get everything lined up
set([ax1,ax2],'Position',[.17 .11 .685 .815]);
cb1 = colorbar(ax1,'Position',[.88 .11 .02 .815],'Ticks',[-2.5,2.5],...
         'TickLabels',{'shoulder','leg'},fontsize=20);
cb2 = colorbar(ax2,'Position',[.88 .28 .02 .515],'Ticks',[-0.1,1.5],...
         'TickLabels',{'moving to arm','moving to leg'},fontsize=20);

cb1.Ruler.TickLabelRotation=90;
cb2.Ruler.TickLabelRotation=90;


%% plot orgine of the leg and shoulder

%origine shoulder:
y1 = -0.4:0.1:2.2; %some x path
x1 = linspace(midpoint-0.25,midpoint-0.25,length(y1));
InterpStyle = 'spline'; %you can choose cubic, nearest, makima, linear, or spline
z1 = interp2(rad2deg(X),rad2deg(Y),Z,rad2deg(x1),rad2deg(y1),'cubic',0); %Cast points(X,Y) on the surface by interpolating Z
hold on
plot3(rad2deg(x1),rad2deg(y1),z1,'-.','LineWidth',2,'Color',[0 0 0], 'DisplayName', 'Origine shoulder') %show the new data

%origine leg
y1 = -0.4:0.1:2.2; %some y path
x1 = linspace(-midpoint+0.25,-midpoint+0.25,length(y1)); %some x path
InterpStyle = 'spline'; %you can choose cubic, nearest, makima, linear, or spline
z1 = interp2(rad2deg(X),rad2deg(Y),Z,rad2deg(x1),rad2deg(y1),'cubic',0); %Cast points(X,Y) on the surface by interpolating Z
hold on
plot3(rad2deg(x1),rad2deg(y1),z1,'-k','LineWidth',2,'Color',[0 0 0], 'DisplayName', 'Origine leg') %show the new data

%% create the path of a scenario (lowering arm, then bending)

%path 1
x1 = midpoint-0.25:0.1:2.6; %some x path
y1 = linspace(-0.1,-0.1,length(x1)); %some y path
InterpStyle = 'spline'; %you can choose cubic, nearest, makima, linear, or spline
z1 = interp2(rad2deg(X),rad2deg(Y),Z,rad2deg(x1),rad2deg(y1),'nearest',0); %Cast points(X,Y) on the surface by interpolating Z
hold on
plot3(rad2deg(x1),rad2deg(y1),z1,'-k','LineWidth',4,'Color',[0 0 1]) %show the new data

%path 2
y1 = -0.1:0.1:0.1; %some x path
x1 = linspace(midpoint-0.25,midpoint-0.25,length(y1)); %some y path
InterpStyle = 'spline'; %you can choose cubic, nearest, makima, linear, or spline
z1 = interp2(rad2deg(X),rad2deg(Y),Z,rad2deg(x1),rad2deg(y1),'cubic',0); %your interpolated z values
hold on
plot3(rad2deg(x1),rad2deg(y1),z1,'-k','LineWidth',4,'Color',[0 0 1]) %show the new data

%path 3
x1 = -midpoint:0.05:midpoint-0.25; %some x path
y1 = linspace(1.5,0.1,length(x1)); %some y path
InterpStyle = 'spline'; %you can choose cubic, nearest, makima, linear, or spline
z1 = interp2(rad2deg(X),rad2deg(Y),Z,rad2deg(x1),rad2deg(y1),'makima',0); %your interpolated z values
hold on
plot3(rad2deg(x1),rad2deg(y1),z1,'-k','LineWidth',4,'Color',[0 0 1]) %show the new data

%path 4
x1 = -midpoint-0.4:0.05:-midpoint; %some x path
y1 = linspace(2.0,1.5,length(x1)); %some y path
InterpStyle = 'spline'; %you can choose cubic, nearest, makima, linear, or spline
z1 = interp2(rad2deg(X),rad2deg(Y),Z,rad2deg(x1),rad2deg(y1),'makima',0); %your interpolated z values
hold on
plot3(rad2deg(x1),rad2deg(y1),z1,'-k','LineWidth',4,'Color',[0 0 1]) %show the new data

%% create 3D labels
ArmDown = text(103,-21,1.2,'Arm down');
set(ArmDown,'BackgroundColor','white','EdgeColor','black',fontsize=15)

ArmUp = text(140,-6,7,'Arm up');
set(ArmUp,'BackgroundColor','white','EdgeColor','black',fontsize=15)

fullyBentdown = text(-131,97,-7,'Fully bent down');
set(fullyBentdown,'BackgroundColor','white','EdgeColor','black',fontsize=15)

slighltyBentDown = text(-102,80,0,'Slighlty bent down');
set(slighltyBentDown,'BackgroundColor','white','EdgeColor','black',fontsize=15)

hold off
xlabel("\textbf{$\theta_{m} (degrees)$}", Interpreter="latex",fontsize=25)
ylabel("$\theta_{imu} (degrees)$", Interpreter="latex",fontsize=25)
zlabel("$\tau_{m} (N.m)$", Interpreter="latex",fontsize=25)
legend({'','Origine shoulder','origine leg','Scenario','','',''}, 'FontSize',20, Location='northeast')










