function [cost,error,location] = costANDerror(uav,obstR,obstX,obstY,optPath)

    cost = 0;
    
    range = sqrt((uav.x-obstX)^2+(uav.y-obstY)^2);
    
    %Calculate cost
    
    %Pentalize for turn around
    if uav.heading > deg2rad(175) && uav.heading <deg2rad(285)
        cost = 1000;
    end
    
    %Pentalize for deviating path
    cost = cost+ abs(uav.y) / (obstR)*uav.dt;
    
    %Pentalize for entering obstacle region
    if range <= obstR
        cost = cost+100;
    end
    
    
    %Calculate error
    [error,location] = calcLatDistance(uav.x,uav.y,[optPath(:,1),optPath(:,2)]);
    
    
    if error<=0.1
        error = 0;
    end

end

