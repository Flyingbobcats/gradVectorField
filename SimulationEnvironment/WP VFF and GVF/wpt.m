classdef wpt
    
    
    properties
        currentWP = 1
        WPx = []
        WPy = []
        WPradius = 7
        active = true
        
        wpx = []
        wpy = []
 
    end
    
    methods
        
        function self = getWPT(self,x,y)
            L = 2*self.WPradius + 10;
            wptx = self.WPx(self.currentWP);
            wpty = self.WPy(self.currentWP);
            
            
            
            dx = wptx - x;                   % change in x between the Waypoint and the UAV
            dy = wpty - y;                   % change in y between the Waypoint and the UAV
            
            alpha = atan2(dy, dx);              % angle between UAV and the current Waypoint measured from the x-axis
            
            A = x + L * cos(alpha);          % current x-position of the Dummy Point
            B = y + L * sin(alpha);         % current y-position of the Dummy point
            
            dA = wptx - A;                   % change in x between the Waypoint and the Dummy Point
            dB = wpty - B;                   % change in y between the Waypoint and the Dummy Point
            
            DUMMYtoWPT = sqrt(dA * dA + dB * dB); % calculate the distance between the Dummy Point and the Waypoint
            UAVtoWPT =   sqrt(dx * dx + dy * dy);   % calculate the distance between the UAV and the Waypoint
            
            if DUMMYtoWPT >= UAVtoWPT          % if the distance from the Dummy Point to the waypoint is greater than
                % the distance from the UAV to the Waypoint...
                if UAVtoWPT <= self.WPradius   % ...and the distance from the UAV to the Waypoint is less than the specified radius
                    if  self.currentWP ~= length(self.WPx)
                        self.currentWP = self.currentWP + 1;  % change to the next Waypoint
                        self.wpx = self.WPx(self.currentWP);
                        self.wpy = self.WPy(self.currentWP);
                    else
                        self.active = false;
                    end
                end
            end
        end
        
        
        
        function self = setup(self,WPs)
            self.WPx = WPs(:,1);
            self.WPy = WPs(:,2);  
            
            self.wpx = self.WPx(1);
            self.wpy = self.WPy(1);
        end
        
        
        function pltWpts(self)
           
            theta = 0:0.01:2.1*pi;
            cxs = self.WPradius*cos(theta);
            cys = self.WPradius*sin(theta);
            for i=1:length(self.WPx)
                plot(cxs+self.WPx(i),cys+self.WPy(i),'r');
                
            end
            
            
            
        end
        
    end
end













    
