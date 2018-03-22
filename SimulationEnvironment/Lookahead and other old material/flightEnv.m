function [xs,ys] = flightEnv(uav,t)


turnrates = -uav.turnrate:0.01:uav.turnrate;

 for j=1:length(turnrates)
        P   = [uav.x;uav.y;0];
        x_b = (uav.v*sin(t*turnrates(j))/turnrates(j));         %ICx in the body frame
        y_b = (uav.v/turnrates(j))*(1-cos(t*turnrates(j)));     %ICy in the body frame
        q   = sqrt(x_b^2+y_b^2);                                %Length of xb,yb
        phi = atan2(y_b,x_b);
        q_b = [q*cos(phi);q*sin(phi);0];
        
        R0_b = [cos(uav.heading)     -sin(uav.heading)      0;
            sin(uav.heading)      cos(uav.heading)      0;
            0                           0               1];
        Q0 = P + R0_b*q_b;
        xs(j) = Q0(1);
        ys(j) = Q0(2);
 end

end