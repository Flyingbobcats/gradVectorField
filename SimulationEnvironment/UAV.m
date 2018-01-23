classdef UAV

    properties
        useDubins = true;
        
        pltHeading = true;
        pltCmdHeading = true;
        
        plotUAV = false;
        plotUAVPath = true;
        
        colorMarker = 'k.';
        headingColor = 'r';
        cmdHeadingColor = 'b';
        
        v = 1;
        dt = 0.1;
        t = 0;
        
        turnrate = 0.35;
        
        %Current state
        x = [];
        y = [];    
        vx = [];
        vy = [];
        heading = [];
        cmdHeading = [];
        
        %History
        xs = [];
        ys = [];
        vxs = [];
        vys = [];
        headings = [];
        headingcmds = [];
        ts = [];
    end
    
    methods
        
        function self =  update_pos(self,vf)
            
            [U,V] = vf.heading(self);
             
            VF_heading = atan2(V,U);
            
            if self.useDubins == true
                theta = atan2(self.vy,self.vx);
                if abs(theta - VF_heading) < pi
                    if theta - VF_heading < 0
                        theta = theta + self.turnrate*self.dt;
                    else
                        theta = theta - self.turnrate*self.dt;
                    end
                else
                    if theta - VF_heading > 0
                        theta = theta + self.turnrate*self.dt;
                    else
                        theta = theta - self.turnrate*self.dt;
                    end
                end
            else
                theta = VF_heading;
            end
            
           
            %Update States
            self.t=self.t+self.dt;
            self.heading = theta;
            self.cmdHeading = VF_heading;
            
            self.vx = self.v*cos(theta);
            self.vy = self.v*sin(theta);
            self.x = self.x+self.vx*self.dt;
            self.y = self.y+self.vy*self.dt;
            
            
            %Update History
            self.xs(end+1) = self.x;
            self.ys(end+1) = self.y;
            self.vxs(end+1) = self.vx;
            self.vys(end+1) = self.vy;
            self.headings(end+1) = theta;
            self.headingcmds(end+1) = VF_heading;
            self.ts(end+1)=self.t;
        end
        
         
        function self = setup(self,x0,y0,v,theta,dt)  
            theta = deg2rad(theta);
            self.x = x0;
            self.y = y0;
            self.v = v;
            self.heading = theta;
            self.vx = v*cos(theta);
            self.vy = v*sin(theta);
            self.dt = dt;
            
            self.xs = x0;
            self.ys = y0;
            self.headings = theta;
            self.headingcmds = theta;
            self.ts = 0;
            
            
        end
        function pltUAV(self)
            
      
            if self.plotUAV
            plot(self.x,self.y,self.colorMarker);
            end
            
            if self.plotUAVPath
                plot(self.xs,self.ys,self.colorMarker);
            end
            
            if self.pltHeading
                U = cos(self.heading);
                V = sin(self.heading);
                quiver(self.x,self.y,U,V,self.headingColor,'linewidth',2,'maxHeadSize',10);
            end
            
            if self.pltCmdHeading
                U = cos(self.cmdHeading);
                V = sin(self.cmdHeading);
                quiver(self.x,self.y,U,V,self.cmdHeadingColor,'linewidth',2,'maxHeadSize',10);
                
                
            end
            
        end
            
            
        
        
        
    end
    
end

