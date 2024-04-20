clear all, clc

[X,Y] = meshgrid(-2.8:0.1:2.8,-0.4:0.1:2.2); %some x and y data

midpoint = 2.04;

As = 3; Al = 2; Ss = 1; L1 = 4.5; L2 = 2.5;
shoulder = As*(1/pi)*atan(rad2deg(X-midpoint)-10) + As/2+0.5;
leg = Al*(1/pi)*atan(-rad2deg(X+midpoint) -3)- As/2 - 2*Ss+0.5; %30 to -3
switching = -4*(1/pi)*atan(rad2deg(Y)-20)+2 ;


Z = shoulder + leg + switching;

s = surf(rad2deg(X),rad2deg(Y),Z,Y,'FaceAlpha',0.3,'EdgeAlpha',0.3,'DisplayName', 'torque'); %surface plot
s.Parent.View = [90 0]; %view the plot

%origine shoulder:
y1 = -0.4:0.1:2.2; %some x path
x1 = linspace(midpoint,midpoint,length(y1)); %some y path
InterpStyle = 'spline'; %you can choose cubic, nearest, makima, linear, or spline
z1 = interp2(rad2deg(X),rad2deg(Y),Z,rad2deg(x1),rad2deg(y1),'cubic',0); %your interpolated z values
hold on
plot3(rad2deg(x1),rad2deg(y1),z1,'-.','LineWidth',2,'Color',[0 0 0], 'DisplayName', 'Origine shoulder') %show the new data

%origine shoulder:
y1 = -0.4:0.1:2.2; %some x path
x1 = linspace(-midpoint,-midpoint,length(y1)); %some y path
InterpStyle = 'spline'; %you can choose cubic, nearest, makima, linear, or spline
z1 = interp2(rad2deg(X),rad2deg(Y),Z,rad2deg(x1),rad2deg(y1),'cubic',0); %your interpolated z values
hold on
plot3(rad2deg(x1),rad2deg(y1),z1,'-k','LineWidth',2,'Color',[0 0 0], 'DisplayName', 'Origine leg') %show the new data

%path 1
x1 = midpoint:0.1:2.6; %some x path
y1 = linspace(-0.1,-0.1,length(x1)); %some y path
InterpStyle = 'spline'; %you can choose cubic, nearest, makima, linear, or spline
z1 = interp2(rad2deg(X),rad2deg(Y),Z,rad2deg(x1),rad2deg(y1),'nearest',0); %your interpolated z values
hold on
plot3(rad2deg(x1),rad2deg(y1),z1,'-k','LineWidth',4,'Color',[0 0 1]) %show the new data

%path 2
y1 = -0.1:0.1:0.1; %some x path
x1 = linspace(midpoint,midpoint,length(y1)); %some y path
InterpStyle = 'spline'; %you can choose cubic, nearest, makima, linear, or spline
z1 = interp2(rad2deg(X),rad2deg(Y),Z,rad2deg(x1),rad2deg(y1),'cubic',0); %your interpolated z values
hold on
plot3(rad2deg(x1),rad2deg(y1),z1,'-k','LineWidth',4,'Color',[0 0 1]) %show the new data

%path 3
x1 = -midpoint:0.05:midpoint; %some x path
y1 = linspace(1.5,0.1,length(x1)); %some y path
InterpStyle = 'spline'; %you can choose cubic, nearest, makima, linear, or spline
z1 = interp2(rad2deg(X),rad2deg(Y),Z,rad2deg(x1),rad2deg(y1),'makima',0); %your interpolated z values
hold on
plot3(rad2deg(x1),rad2deg(y1),z1,'-k','LineWidth',4,'Color',[0 0 1]) %show the new data

%path 4
x1 = -midpoint-0.6:0.05:-midpoint; %some x path
y1 = linspace(2.0,1.5,length(x1)); %some y path
InterpStyle = 'spline'; %you can choose cubic, nearest, makima, linear, or spline
z1 = interp2(rad2deg(X),rad2deg(Y),Z,rad2deg(x1),rad2deg(y1),'makima',0); %your interpolated z values
hold on
plot3(rad2deg(x1),rad2deg(y1),z1,'-k','LineWidth',4,'Color',[0 0 1]) %show the new data


ArmDown = text(50,-21,0.7,'Arm down');
set(ArmDown,'BackgroundColor','white','EdgeColor','black',fontsize=15)

ArmUp = text(160,-21,3.8,'Arm up');
set(ArmUp,'BackgroundColor','white','EdgeColor','black',fontsize=15)

fullyBentdown = text(-150,80,-1.3,'Fully bent down');
set(fullyBentdown,'BackgroundColor','white','EdgeColor','black',fontsize=15)

slighltyBentDown = text(23,108,-3.3,'Slighlty bent down');
set(slighltyBentDown,'BackgroundColor','white','EdgeColor','black',fontsize=15)

hold off
xlabel("\textbf{$\theta_{m} (degrees)$}", Interpreter="latex",fontsize=25)
ylabel("$\theta_{imu} (degrees)$", Interpreter="latex",fontsize=25)
zlabel("$\tau_{m} (N.m)$", Interpreter="latex",fontsize=25)
legend({'','Origine shoulder','origine leg','','','',''}, 'FontSize',20, Location='best')
s.Parent.View = [12 25]; %you can change the view to see what the black line looks like in 3D


mymap = [1 0 0
    0 1 0
    0 0 1
    0 0 1
    0 0 1
    0 0 0.8];

colormap(mymap);
c =colorbar('Ticks',[-0.2,0.2,1.5],...
         'TickLabels',{'shoulder','slack','leg'},fontsize=20)
c.Label.String = 'assistance to';



