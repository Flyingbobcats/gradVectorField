

clc
clear
close all

turnRate = 0.35;
turnR = 1;
obstR = 10;


obstX = 0;
obstY = 0;

y = turnR*(1-cos(pi/2));
X = obstX - sqrt((turnR+obstR)^2-(y-obstY)^2);
xs = linspace(-(obstR+turnR)*1.5,X,10);
ys = zeros(1,length(xs));

%Angles
d_angle = deg2rad(1);

theta = asin((y-obstY)/(obstR+turnR));
beta = 3*pi/2:d_angle:2*pi-theta;
gamma = theta:d_angle:pi-theta;
zeta = pi+theta:d_angle:3*pi/2;



optPathX = [xs,obstX + obstR*cos(gamma),X+turnR*cos(beta),-X+turnR*cos(zeta), -xs];
optPathY = [ys,obstY + obstR*sin(gamma),y+turnR*sin(beta), y+turnR*sin(zeta),  ys];

hold on




%Straight Path
p1 = plot(xs,ys,'-k');
plot(-xs,ys,'-k');

%Obstacle to be avoided
p2 = plot(obstX+obstR*cos(0:0.01:2*pi),obstY+obstR*sin(0:0.01:2*pi),'--','linewidth',4);

%Avoidance Path
% p3 = plot(obstX + obstR*cos(theta:0.001:pi-theta),obstY + obstR*sin(theta:0.001:pi-theta),'r','linewidth',2);
% plot([X+turnR*cos(beta)],[y+turnR*sin(beta)],'r','linewidth',2);
% plot([-X+turnR*cos(pi+theta:0.01:3*pi/2)],[y+turnR*sin(pi+theta:0.01:3*pi/2)],'r','linewidth',2);
plot(optPathX,optPathY,'k.');

%Helpers
p4 = plot(X,y,'b*');
 plot(-X,y,'b*');
p5 = plot([X,X+turnR*cos(beta(end))],[y,y+turnR*sin(beta(end))],[X,X+turnR*cos(beta(1))],[y,y+turnR*sin(beta(1))]);



axis equal
xlabel('x');
ylabel('y');
grid on




