function optPath = genOptPath(uav,obstR,obstX,obstY)

    turnRate = uav.turnrate;
    turnR = uav.turn_radius;
    obstR = obstR;
    
%   turnRate = 0.35;
%   turnR = 10/turnRate;
%   obstR = turnR;
%   obstX = 0;
%   obstY = 0;




    y = turnR;
    X = obstX - sqrt((turnR+obstR)^2-(y-obstY)^2);
    xs = linspace(-(obstR+turnR)*1.5,X,50);
    ys = zeros(1,length(xs));

    %Angles
    d_angle = deg2rad(0.2);

    theta = asin((y-obstY)/(obstR+turnR));
    beta = 3*pi/2:d_angle:2*pi-theta;
    
    gamma = theta:d_angle:pi-theta;
    gamma = pi-theta:-d_angle:theta;
    
    zeta = pi+theta:d_angle:3*pi/2;
        
    
%     optPathX = [xs,X+turnR*cos(beta),obstX + obstR*cos(gamma),-X+turnR*cos(zeta), -xs];
%     optPathY = [ys,y+turnR*sin(beta),obstY + obstR*sin(gamma), y+turnR*sin(zeta),  ys];
    
    optPathX = [X+turnR*cos(beta),obstX + obstR*cos(gamma),-X+turnR*cos(zeta)];
    optPathY = [y+turnR*sin(beta),obstY + obstR*sin(gamma), y+turnR*sin(zeta)];    

    optPath = [optPathX',optPathY'];
    
    
    

    


end

