
classdef gvfLine  
    properties
        active = true
        normComponents = true
        
        %Cylinder properties
        x = 0;
        y = 0;
        r = 3;
        
   
        vx = 0;
        vy = 0;
        
        %Plane Properties
        z = 1;
        
        %Weights
        G = 1;
        H = 1;
        
        %Space properties
        n = 25;
        xspace = linspace(-10,10,25);
        yspace = linspace(-10,10,25);
        LineDistance = 5;
        
        %Define the surfaces
        angle = 0

        
        %Define the gradiants of the surfaces
        g1 = @(x,y) [0*0.5;1;0];
        g2 = @() [0;0;1];


        %Activation Function
        decay = @(r) 1
        act = 1;
        ext = 1;
    end
    

    
    methods 
        
        function [u,v] = comp(self,posx,posy)
            
            range = 1;
            
            a = 1*cos(self.angle);
            b = 1*sin(self.angle);
            a1 = a*(posx-self.x)+b*(posy-self.y);
            ga1 = [a;b;0];
            a2 = 1;
            ga2 = [0;0;1];
            
            Vconv = (a1*ga1+a2*ga2);
            Vcirc = cross(ga1,ga2);

            
            
            if self.normComponents == true
                VcircNorm = norm(Vcirc);
                VconvNorm = norm(Vconv);
                Vt = -self.G*Vconv/VconvNorm+self.H*Vcirc/VcircNorm;
            else
                Vt = -self.G*Vconv+self.H*Vcirc;
            end
            
            mag = norm(Vt);
            u =   self.decay(range)*Vt(1)/mag;
            v =   self.decay(range)*Vt(2)/mag;

        end
      
        
        function [X,Y,U,V] = ff(self) %Output entire vector field
            U = NaN(length(self.xspace),length(self.yspace));
            V = NaN(length(self.xspace),length(self.yspace));
            X = NaN(length(self.xspace),length(self.yspace));
            Y = NaN(length(self.xspace),length(self.yspace));
            for i = 1:length(self.xspace)
                for j = 1:length(self.yspace)
                    [u,v] = comp(self,self.xspace(i),self.yspace(j));
                    vec = [u,v];
                    mag = sqrt(vec(1)^2+vec(2)^2);
                    R = sqrt((self.xspace(i)-self.x)^2+(self.yspace(j)-self.y)^2);
                    U(i,j) =  vec(1)/mag;
                    V(i,j) =  vec(2)/mag;
                    X(i,j) = self.xspace(i);
                    Y(i,j) = self.yspace(j);
                end
            end
        end
        
        
        
        
        
        % = = = = = = = = Plotting Functions = = = = = = = = = =  %
        
        function pltfnc(self)
           
           d = [-self.LineDistance/2,self.LineDistance/2];
           cxs = self.x+d*cos(self.angle-pi/2);
           cys = self.y+d*sin(self.angle-pi/2);
           plot(cxs,cys,'r','linewidth',2);
        end
        
        function pltcndr(self)
            th = 0:0.05:2.1*pi;
            cxs = self.x+self.r*cos(th);
            cys = self.y+self.r*sin(th);
            plot(cxs,cys,'r','linewidth',3);
        end
        
        
    end
end
