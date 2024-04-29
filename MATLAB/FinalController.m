clear all
clf

%motor position/ imu angle
x = linspace(-pi,1.1*pi,1000);
imu = linspace(-3,3,1000);

%torque amplitude
As = 2;
Al = 2;
Ss = 1;

%reduction ratio
Rpulley = 57; %mm
l = 20;
delatCable = l*(1+cos(pi/4));
delatThetaMot = delatCable/Rpulley;
delatArm = deg2rad(150);
R = deg2rad(90)/0.2%delatArm/delatThetaMot


rangeshoulder = linspace(-pi,pi/6.6,1000);



%orgine leg and shoulder
zero_leg = 4.0;
zero_shoulder = -0.07;
midpoint = abs(zero_leg-zero_shoulder)/2;

%torque
Ts = As*sin((rangeshoulder-zero_shoulder)*R);
Tleg = -As*sin(imu);

Switch_leg = (1/pi)*atan(rad2deg(imu) -3)+0.5; 
Switch_shoulder = ((1/pi)*atan(rad2deg((rangeshoulder-zero_shoulder)*R) -3)+0.5);
Ts_switch = Ts.*((1/pi)*atan(rad2deg((rangeshoulder-zero_shoulder)*R) -3)+0.5) +0.5;
Tleg_switch = Tleg.*Switch_leg;



subplot(3,4,1)
plot(rad2deg(imu),Switch_leg,LineWidth=3)
title("$\Gamma_{Leg}= \frac{1}{\pi}*atan(\theta_{imu}) +0.5$",Interpreter="latex",FontSize=20)
xlabel("$\theta_{imu} (degrees)$",Interpreter="latex",FontSize=15)
ylabel("ON/OFF",Interpreter="latex",FontSize=15)
xregion(0,200,"FaceColor", 'r');
xregion(-200,0,"FaceColor", [0.2 0.2 0.2]);
txt1 = {'Leg OFF'};
text(-180,0.5,txt1, fontsize=15)
txt2 = {'Leg ON'};
text(50,0.5,5,txt2,fontsize=15)


subplot(3,4,2)
plot(rad2deg(rangeshoulder),Switch_shoulder,LineWidth=3)
title("$\Gamma_{shoulder}= \frac{1}{\pi}*atan(\theta_{mot}-\theta_{shoudler}^{origine})*R +0.5$",Interpreter="latex",FontSize=20)
xlabel("$\theta_{mot}-\theta_{shoudler}^{origine}$ (degrees)",Interpreter="latex",FontSize=15)
ylabel("ON/OFF",Interpreter="latex",FontSize=15)
xregion(0,200,"FaceColor", 'b');
xregion(-200,0,"FaceColor",[0.2 0.2 0.2] );
txt1 = {'Shoulder OFF'};
text(-180,0.5,txt1, fontsize=15)
txt2 = {'Shoulder ON'};
text(50,0.5,5,txt2,fontsize=15)



subplot(3,4,5)
plot(rad2deg(imu),Tleg,LineWidth=3)
title("$\tau_{leg}= -As*sin(\theta_{imu}) $",Interpreter="latex",FontSize=20)
xlabel("$\theta_{imu} (degrees)$",Interpreter="latex",FontSize=15)
ylabel("$\tau_{mot} (N.m)$",Interpreter="latex",FontSize=15)
xregion(0,200,"FaceColor", 'r');
xregion(-200,0,"FaceColor", [0.2 0.2 0.2]);
txt1 = {'Leg OFF'};
text(-180,0,txt1, fontsize=15)
txt2 = {'Leg ON'};
text(50,0,txt2,fontsize=15)




subplot(3,4,6)
plot(rad2deg(rangeshoulder),Ts,LineWidth=3)
title("$ \tau_{shoulder} = A_{arm}.sin((\theta_{mot} -\theta_{origine}^{arm})R) $",Interpreter="latex",FontSize=20)
xlabel("$\theta_{mot} -\theta_{shoudler}^{origine}$ (degrees)",Interpreter="latex",FontSize=15)
ylabel("$\tau_{mot} (N.m)$",Interpreter="latex",FontSize=15)
xregion(-200,0,"FaceColor", [0.2 0.2 0.2]);
xregion(0,200,"FaceColor",'b' );
txt1 = {'Shoulder OFF'};
text(-150,0,txt1, fontsize=15)
txt2 = {'Shoulder ON'};
text(50,0,5,txt2,fontsize=15)


subplot(3,4,9)
plot(rad2deg(imu),Tleg_switch,LineWidth=3)
title("$\tau_{leg}.\Gamma_{Leg}$",Interpreter="latex",FontSize=20)
xlabel("$\theta_{imu} (degrees)$",Interpreter="latex",FontSize=15)
ylabel("$\tau_{mot} (N.m)$",Interpreter="latex",FontSize=15)
xregion(0,200,"FaceColor", 'r');
xregion(-200,0,"FaceColor", [0.2 0.2 0.2]);
txt1 = {'Leg OFF'};
text(-180,-1,txt1, fontsize=15)
txt2 = {'Leg ON'};
text(50,-1,txt2,fontsize=15)


subplot(3,4,10)
plot(rad2deg(rangeshoulder),Ts_switch,LineWidth=3)
title("$\tau_{shoulder}.\Gamma_{Shoulder}$",Interpreter="latex",FontSize=20)
xlabel("$\theta_{mot} -\theta_{shoudler}^{origine}$ (degrees)",Interpreter="latex",FontSize=15)
ylabel("$\tau_{mot} (N.m)$",Interpreter="latex",FontSize=15)
xregion(-200,0,"FaceColor", [0.2 0.2 0.2]);
xregion(0,100,"FaceColor",'b' );
txt1 = {'Shoulder OFF'};
text(-150,1,txt1, fontsize=15)
txt2 = {'Shoulder','ON'};
text(30,1,5,txt2,fontsize=15)




%%%%%%%%%%%%%%%%%%%%% 3d graph %%%%%%%%%%%%%%%

%colormaps
mymap1 = [0 0 0.7
    0 0 0.7
    0 0 0.7
    0 0 0.7
    0 0 0.7
    0.7 0 0
    0.7 0 0
    0.7 0 0
    0.7 0 0
    0.7 0 0
    0.7 0 0
    0.7 0 0
    0.7 0 0
    0.7 0 0
    0.7 0 0
    0.7 0 0
    0.7 0 0
    0.7 0 0
    0.7 0 0];

mymap2 = [0 1 0
    0 0.5 0
    0 0.5 0];


%% plot surfaces



[X,Y] = meshgrid(-pi:0.08:pi,-0.5:0.06:2.2); %some x and y data
%torque
Ts = As*sin((X-zero_shoulder)*R);
Tleg = As*sin(Y);

Switch_leg = (1/pi)*atan(rad2deg(Y) -3)+0.5; 
Switch_shoulder = 1-((1/pi)*atan(rad2deg(Y)-10)+0.5);

Ts_switch = Ts.*Switch_shoulder.*(1-((1/pi)*atan(rad2deg((X-zero_shoulder)*R) -3)+0.5));
Tleg_switch = Tleg.*Switch_leg;

switching = -0.5*(1/pi)*atan(rad2deg(Y)-20)-0.1;

Z = Ts_switch+Tleg_switch-0.5;



subplot(3,4,[3 4 7 8 11 12])
surf(rad2deg(X),rad2deg(Y),Z,Y,'FaceAlpha',0.4,'EdgeAlpha',0.6,'DisplayName', 'torque'); %surface plot
ax1.Visible = 'on'
xlabel("\textbf{$\theta_{m} (degrees)$}", Interpreter="latex",fontsize=25)
ylabel("$\theta_{imu} (degrees)$", Interpreter="latex",fontsize=25)
zlabel("$\tau_{m} (N.m)$", Interpreter="latex",fontsize=25)

title("$\tau_{motor} = \tau_{shoulder}.\Gamma_{Shoulder}+ \tau_{Leg}.\Gamma_{Leg}$",Interpreter="latex",FontSize=30)
colormap(mymap1)

%% plot orgine of the leg and shoulder

%origine shoulder:
y1 = -0.5:0.01:deg2rad(12); %some x path
x1 = linspace(zero_shoulder,zero_shoulder,length(y1));
InterpStyle = 'spline'; %you can choose cubic, nearest, makima, linear, or spline
z1 = interp2(rad2deg(X),rad2deg(Y),Z,rad2deg(x1),rad2deg(y1),'cubic',0); %Cast points(X,Y) on the surface by interpolating Z
hold on
plot3(rad2deg(x1),rad2deg(y1),z1,'-.','LineWidth',4,'Color',[0 0 0], 'DisplayName', 'Origine shoulder') %show the new data

%imu threshold
x1 = zero_leg-0.4:0.01:(zero_shoulder+pi)/R+0.1; %some y path
y1 = linspace(12,12,length(x1)); %some x path
InterpStyle = 'spline'; %you can choose cubic, nearest, makima, linear, or spline
z1 = interp2(rad2deg(X),rad2deg(Y),Z,rad2deg(x1),y1,'cubic',0); %Cast points(X,Y) on the surface by interpolating Z
hold on
plot3(rad2deg(x1),y1,z1,'-k','LineWidth',4,'Color',[0 0 0], 'DisplayName', 'Origine leg') %show the new data

%% create the path of a scenario (lowering arm, then bending)

%path 1
x1 = zero_shoulder:0.01:0.3; %some x path
y1 = linspace(0,0,length(x1)); %some y path
InterpStyle = 'spline'; %you can choose cubic, nearest, makima, linear, or spline
z1 = interp2(rad2deg(X),rad2deg(Y),Z,rad2deg(x1),rad2deg(y1),'cubic',0); %Cast points(X,Y) on the surface by interpolating Z
hold on
plot3(rad2deg(x1),rad2deg(y1),z1,'-k','LineWidth',4,'Color',[1 1 0]) %show the new data
plot3(14,0,2.4,'o',MarkerFaceColor=[1 0 1],MarkerEdgeColor=[0 0 0], MarkerSize= 10)
ArmDown = text(14,0,2.6,'1',color='magenta');
set(ArmDown,'BackgroundColor','none','EdgeColor','none',fontsize=30)
%path 2
y1 = 0:0.01:deg2rad(12); %some x path
x1 = linspace(zero_shoulder,zero_shoulder,length(y1)); %some y path
InterpStyle = 'spline'; %you can choose cubic, nearest, makima, linear, or spline
z1 = interp2(rad2deg(X),rad2deg(Y),Z,rad2deg(x1),rad2deg(y1),'cubic',0); %your interpolated z values
hold on
plot3(rad2deg(x1),rad2deg(y1),z1+0.04,'-k','LineWidth',4,'Color',[1 1 0])
plot3(4,0,0.59,'o',MarkerFaceColor=[1 0 1],MarkerEdgeColor=[0 0 0], MarkerSize= 10)
ArmDown = text(14,-5,0.7,'2',color='magenta');
set(ArmDown,'BackgroundColor','none','EdgeColor','none',fontsize=30)

%path 3
x1 = zero_leg+0.1:0.01:zero_shoulder; %some x path
y1 = linspace(deg2rad(20),deg2rad(12),length(x1)); %some y path
InterpStyle = 'spline'; %you can choose cubic, nearest, makima, linear, or spline
z1 = interp2(rad2deg(X),rad2deg(Y),Z,rad2deg(x1),rad2deg(y1),'makima',0); %your interpolated z values
hold on
plot3(rad2deg(x1),rad2deg(y1),z1,'--','LineWidth',4,'Color',[1 1 0]) %show the new data
plot3(3.43,12,0.1,'o',MarkerFaceColor=[1 0 1],MarkerEdgeColor=[0 0 0], MarkerSize= 10)
ArmDown = text(3.43,30,0.1,'3',color='magenta');
set(ArmDown,'BackgroundColor','none','EdgeColor','none',fontsize=30)

%path 4
x1 = linspace(zero_leg+0.1,zero_leg+0.1+deg2rad(87),1000); %some x path
y1 = linspace(deg2rad(20),deg2rad(100),length(x1)); %some y path
InterpStyle = 'spline'; %you can choose cubic, nearest, makima, linear, or spline
z1 = interp2(rad2deg(X),rad2deg(Y),Z,rad2deg(x1),rad2deg(y1),'makima',0); %your interpolated z values
hold on
plot3(rad2deg(x1),rad2deg(y1),z1,'-k','LineWidth',4,'Color',[1 1 0]) %show the new data
plot3(-220,19.48,-0.11,'o',MarkerFaceColor=[1 0 1],MarkerEdgeColor=[0 0 0], MarkerSize= 10)
ArmDown = text(-200,30.48,-0.11,'4',color='magenta');
set(ArmDown,'BackgroundColor','none','EdgeColor','none',fontsize=30)


%path 5
x1 = linspace(zero_leg-0.2,zero_leg+0.1+deg2rad(87),1000); %some x path
y1 = linspace(deg2rad(12),deg2rad(100),length(x1)); %some y path
InterpStyle = 'spline'; %you can choose cubic, nearest, makima, linear, or spline
z1 = interp2(rad2deg(X),rad2deg(Y),Z,rad2deg(x1),rad2deg(y1),'makima',0); %your interpolated z values
hold on
plot3(rad2deg(x1),rad2deg(y1),z1,'-k','LineWidth',5,'Color',[0 1 0]) %show the new data
plot3(-151,84,-1.47,'o',MarkerFaceColor=[1 0 1],MarkerEdgeColor=[0 0 0], MarkerSize= 10)
ArmDown = text(-101,84,-1.57,'5',color='magenta');
set(ArmDown,'BackgroundColor','none','EdgeColor','none',fontsize=30)

%path 6
x1 = linspace(zero_leg-0.2,zero_shoulder,1000); %some x path
y1 = linspace(deg2rad(12),deg2rad(0),length(x1)); %some y path
InterpStyle = 'spline'; %you can choose cubic, nearest, makima, linear, or spline
z1 = interp2(rad2deg(X),rad2deg(Y),Z,rad2deg(x1),rad2deg(y1),'makima',0); %your interpolated z values
hold on
plot3(rad2deg(x1),rad2deg(y1),z1,'--','LineWidth',5,'Color',[0 1 0]) %show the new data
plot3(-240,12.17,0.1,'o',MarkerFaceColor=[1 0 1],MarkerEdgeColor=[0 0 0], MarkerSize= 10)
ArmDown = text(-250,8,0.2,'6',color='magenta');
set(ArmDown,'BackgroundColor','none','EdgeColor','none',fontsize=30)






%% create 3D labels




legend({'','$\theta_{Shoulder}=0^{\circ}$','$\theta_{imu}=12^{\circ}$','going down','','','','going up'},Interpreter='Latex',FontSize=20, Location='northwest')
hold off

cc = colorbar('Location','manual',FontSize=20);
cc.Position = [0.92 0.35 .03 .5]
cc.YTick = [-0.2 1.2];
cc.YTickLabel = {'Shoulder', 'Leg'};

view(-120,30)
