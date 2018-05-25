classdef UAV

    properties
        useDubins = true;
        
        plotHeading = true;
        plotCmdHeading = true;
        
        plotUAV = false;
        plotUAVPath = true;
        plotFlightEnv = true;
        
        colorMarker = 'k.';
        headingColor = 'r';
        cmdHeadingColor = 'b';
        
        v = 1;
        dt = 0.1;
        t = 0;
        tLookAhead = 4.5;
        
        turnrate = 0.35;
        turn_radius = [];
        
        %Current state
        x = [];
        y = [];    
        vx = [];
        vy = [];
        heading = [];
        cmdHeading = [];
        flightEnvX = [];
        flightEnvY = [];
        
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
        
        function self =  update_pos(self,heading)     
            VF_heading = heading;
            
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
            
            %Update flight envelope
            self = self.calcFlightEnv();
            
            
            %Update History
            self.xs(end+1) = self.x;
            self.ys(end+1) = self.y;
            self.vxs(end+1) = self.vx;
            self.vys(end+1) = self.vy;
            self.headings(end+1) = theta;
            self.headingcmds(end+1) = VF_heading;
            self.ts(end+1)=self.t;
        end
        
         
        
        function self = calcFlightEnv(self)
            
             
%             turnrates = -self.turnrate:0.05:self.turnrate;
            
            turnrates = linspace(-self.turnrate,self.turnrate,5);
            
            for j=1:length(turnrates)
                
                if turnrates(j) ~=0
                P   = [self.x;self.y;0];
                x_b = (self.v*sin(self.tLookAhead*turnrates(j))/turnrates(j));         %ICx in the body frame
                y_b = (self.v/turnrates(j))*(1-cos(self.tLookAhead*turnrates(j)));     %ICy in the body frame
                q   = sqrt(x_b^2+y_b^2);                                               %Length of xb,yb
                phi = atan2(y_b,x_b);
                q_b = [q*cos(phi);q*sin(phi);0];
                
                R0_b = [cos(self.heading)     -sin(self.heading)      0;
                        sin(self.heading)      cos(self.heading)      0;
                        0                           0               1];
                
                    
                Q0 = P + R0_b*q_b;
                self.flightEnvX(j) = Q0(1);
                self.flightEnvY(j) = Q0(2);
                
                else
                    P   = [self.x;self.y;0];
                    x_b = (self.v*self.tLookAhead);         %ICx in the body frame
                    y_b = 0;                                                                %ICy in the body frame
                    q   = sqrt(x_b^2+y_b^2);                                                %Length of xb,yb
                    phi = atan2(y_b,x_b);
                    q_b = [q*cos(phi);q*sin(phi);0];
                    
                    R0_b = [cos(self.heading)     -sin(self.heading)      0;
                        sin(self.heading)      cos(self.heading)      0;
                        0                           0               1];
     
                    Q0 = P + R0_b*q_b;
                    self.flightEnvX(j) = Q0(1);
                    self.flightEnvY(j) = Q0(2);
                end
                    
            end
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
            
            self.turn_radius = self.v/self.turnrate;
            
            
        end
        function fig = pltUAV(self)
            
            
            if self.plotUAV
            fig = plot(self.x,self.y,self.colorMarker);
            end
            
            if self.plotUAVPath
                fig = plot(self.xs,self.ys,self.colorMarker,'linewidth',5);
            end
            
            if self.plotHeading
                U = cos(self.heading);
                V = sin(self.heading);
                quiver(self.x,self.y,U,V,self.headingColor,'linewidth',2,'maxHeadSize',10);
            end
            
            if self.plotCmdHeading
                U = cos(self.cmdHeading);
                V = sin(self.cmdHeading);
                quiver(self.x,self.y,U,V,self.cmdHeadingColor,'linewidth',2,'maxHeadSize',10); 
            end
            
            
            if self.plotFlightEnv
                
                X1 = [self.x,self.flightEnvX(end)];
                Y1 = [self.y,self.flightEnvY(end)];
                
                X2 = [self.x,self.flightEnvX(1)];
                Y2 = [self.y,self.flightEnvY(1)];
                
                
                plot(self.flightEnvX,self.flightEnvY,'r.',X1,Y1,'r',X2,Y2,'r');
            end
 
        end
            
            
        
        
        
    end
    
end

