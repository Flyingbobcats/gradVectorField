function optPath = genOptPath(uav,obstR)

    turnRate = uav.turnrate;
    turnR = uav.turn_radius;
    obstR = obstR;


    obstX = 0;
    obstY = 0;

    y = turnR*(1-cos(pi/2));
    X = obstX - sqrt((turnR+obstR)^2-(y-obstY)^2);
    xs = linspace(-(obstR+turnR)*1.5,X,50);
    ys = zeros(1,length(xs));

    %Angles
    d_angle = deg2rad(0.2);

    theta = asin((y-obstY)/(obstR+turnR));
    beta = 3*pi/2:d_angle:2*pi-theta;
    gamma = theta:d_angle:pi-theta;
    zeta = pi+theta:d_angle:3*pi/2;



    optPathX = [xs,obstX + obstR*cos(gamma),X+turnR*cos(beta),-X+turnR*cos(zeta), -xs];
    optPathY = [ys,obstY + obstR*sin(gamma),y+turnR*sin(beta), y+turnR*sin(zeta),  ys];

    optPath = [optPathX',optPathY'];
    
    

    


end

