function [dist,index] = calcLatDistance(x,y,path)
    distanceToPath = NaN(1,length(path));
    for i = 1:length(path)
        distanceToPath(i) = sqrt((x-path(i,1))^2+(y-path(i,2))^2);  
    end
    [dist,index] = min(distanceToPath);
%     location = [path(point,1),path(point,2)];
    
end

