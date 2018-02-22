%IC points

function [XYS] = getICPoints(type,varargin)



if ~isempty(varargin)
    options = varargin{1};
end

if strcmp(type,'grid')
    
    if isempty(varargin)
        n = 10;
        m = 10;
        xlimit = 10;
        ylimit = 10;
        
    else
        n = options.n;
        m = options.m;
        xlimit = options.xlimit;
        ylimit = options.ylimit;
    end
    x_row = linspace(-xlimit,xlimit,n);
    yrow = linspace(-ylimit,ylimit,m);
    
    
    %Remove origin
    
    XS = [];
    YS = [];
    for i=1:m 
        XS = [XS,x_row];
        YS = [YS,yrow(i)*ones(1,n)]; 
    end
    

end


if strcmp(type,'circle')
   
    if ~isempty(varargin)
        r = options.r;
        d_theta = options.d_theta;
        
    else
        r = 10;
        d_theta = 0.1;
    end
    
    theta = 0:d_theta:2*pi;
    XS = r*cos(theta);
    YS = r*sin(theta);
    
    
end

XYS = [XS;YS];


end











