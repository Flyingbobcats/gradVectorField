
function path_heading = loadPath(options,t)


    if strcmp(options,'horiz')
        path_heading = deg2rad(0);
        
    elseif strcmp(options,'vert')
        path_heading = deg2rad(90);
        
    elseif strcmp(options,'pulse')
        n = 50;
        if t<=1*n
            path_heading = deg2rad(0);
            
        elseif t<=2*n && t>1*n
            path_heading = deg2rad(90);
        elseif t<=3*n && t>2*n
            path_heading = deg2rad(0);
        elseif t<=4*n && t>3*n
            path_heading = deg2rad(-90);
        elseif t>4*n
            path_heading = deg2rad(0);
        end
        
        
    else
        str = strcat(options, ' is not a valid input argument');
        warning(str);
    end
    
    
end