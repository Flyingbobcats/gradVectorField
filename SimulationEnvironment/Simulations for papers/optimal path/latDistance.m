%Laterial distance

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

posx = -20;
posy = 0.5;

while posx < 20
    posx = posx+0.1;
    
    [dist,location] = calcLatDistance(posx,posy,[optPathX',optPathY']);

    
    clf
    hold on
    plot([posx,location(1)],[posy,location(2)],'g');
    plot(optPathX,optPathY,'k.');
    plot(posx,posy,'b*');
    title(num2str(dist));
        axis([-20,20,-20,20]);
        grid on
        axis equal
    pause()

    
    
end
