%Simnple cylinder and plane VF with decayivation functions
classdef cndr
    properties
        active = true
        normComponents = true
        normTotal = true
        decayActive = true
        warningCheckNorm = false
        
        %Cylinder properties
        x = 0;
        y = 0;
        r = 3;
        vx = 0
        vy = 0;
        e  =0.5;
        
        const
        %Plane Properties
        z = 1;
        
        %Space properties
        n = 50;
        xspace = linspace(-35,35,50);
        yspace = linspace(-35,35,50);
        
        localWidth = 10;
        localHeight = 10;
        
        %Define the surfaces
        a1 = @(x,y,r) x^2+y^2-r^2;
        a2 = @(z) z;
        
        %Define the gradiants of the surfaces
        g1 = @(x,y) [2*x;2*y;0];
        g2 = @() [0;0;1];
        
        
        %Activation Functions
        H = 0
        G = 1
        L = 0
        
        type = 'const'
        
        %Decay Function
        decayR = 5
        decay = @(r) 1;
        ext = 1;
    end
    
    
    
    methods
        
        
        function [u,v] = comp(self,posx,posy)
            
            %Determine G and H
            range = sqrt((self.x-posx)^2+(self.y-posy)^2);
            [self.G,self.H] = getGH(self,range);
            
            %Calculate the field components
            Vconv = ((self.a1(posx-self.x,posy-self.y,self.r)*self.g1(posx-self.x,posy-self.y)) + self.a2(self.z)*self.g2());
            Vcirc = (cross(self.g1(posx-self.x,posy-self.y),self.g2()));
            
            Vtv = ((-2*self.vx*(posx-self.x)-2*self.vy*(posy-self.y) ) / ((2*(posx-self.x))^2 + (2*(posy-self.y))^2) )*[2*(posx-self.x); 2*(posy-self.y);0];
            Vtv = Vtv*-1;
            %Normalize each components
            if self.normComponents == true
                
                VcircNorm = sqrt(Vcirc(1)^2+Vcirc(2)^2);
                VconvNorm = sqrt(Vconv(1)^2+Vconv(2)^2);
                
                VtvNorm =  sqrt(Vtv(1)^2+Vtv(2)^2);
                
%                 VcircNorm = norm(Vcirc);
%                 VconvNorm = norm(Vconv);
%                 VtvNorm = norm(Vtv);
                
                if VtvNorm == 0
                     Vt = -self.G*Vconv/VconvNorm+self.H*Vcirc/VcircNorm;
                else

                Vt = -self.G*Vconv/VconvNorm+self.H*Vcirc/VcircNorm+self.L*Vtv/VtvNorm;
                end
            else
                Vt = -self.G*Vconv+self.H*Vcirc+self.L*Vtv;
            end
            
            
            
            
            
            
            %Normalize the total
            if self.normTotal
                N = sqrt(Vt(1)^2+Vt(2)^2);
                u =   Vt(1)/N;
                v =   Vt(2)/N;
            else
                u = Vt(1);
                v = Vt(2);
            end
            
            %Apply the decay function
            if self.decayActive
%                 p = self.decay(range);
                p = -(tanh(2*pi*range/self.decayR-pi))+1;
%                 if range<2
%                     p
%                 end
                u = p*u;
                v = p*v;
            end
            
            %Warning if the output is not a normalized vector
            if self.warningCheckNorm
                N = norm([u,v]);
                if N >1
                    warning('Output is not a normalized vector');
                end
            end
        end
        
        
        
        
        function [X,Y,U,V] = ff(self)
            U = NaN(length(self.xspace),length(self.yspace));
            V = NaN(length(self.xspace),length(self.yspace));
            X = NaN(length(self.xspace),length(self.yspace));
            Y = NaN(length(self.xspace),length(self.yspace));
            for i = 1:length(self.xspace)
                for j = 1:length(self.yspace)
                    [u,v] = comp(self,self.xspace(i),self.yspace(j));
                    vec = [u,v];
                    U(i,j) =  vec(1);
                    V(i,j) =  vec(2);
                    X(i,j) = self.xspace(i);
                    Y(i,j) = self.yspace(j);
                end
            end
        end
        
        
        %Used for plotting a field centered at (x,y)
        function [X,Y,U,V] = ffLocal(self)
            self.localWidth = self.decayR;
            self.localHeight = self.decayR;
            xspacelocal = linspace(-self.localWidth,self.localWidth,self.n)+self.x;
            yspacelocal = linspace(-self.localHeight,self.localHeight,self.n)+self.y;
            U = NaN(length(xspacelocal),length(yspacelocal));
            V = NaN(length(xspacelocal),length(yspacelocal));
            X = NaN(length(xspacelocal),length(yspacelocal));
            Y = NaN(length(xspacelocal),length(yspacelocal));
            for i = 1:length(xspacelocal)
                for j = 1:length(yspacelocal)
                    [u,v] = comp(self,xspacelocal(i),yspacelocal(j));
                    vec = [u,v];
                    U(i,j) =  vec(1);
                    V(i,j) =  vec(2);
                    X(i,j) = xspacelocal(i);
                    Y(i,j) = yspacelocal(j);
                end
            end
        end
        
        
        
        
        % ===================== Activation and Decay Functions ===================
        function [G,H] = getGH(self,range)
            
            if strcmp(self.type,'const')
                G = self.G;
                H = self.H;
            end
            if strcmp(self.type,'channel')
                if range > self.r+self.e %|| range < self.r-self.e
                    G = 1;
                    H = 1;
                elseif range < self.r-self.e
                    H = 0;
                    G = self.G;
                else
                    G = 0.5/range;
                    H = 1;
                end
            end
        end
        
        function self = modDecay(self,type)
            if strcmp(type,'none') == 1
                self.decay = @(r) 1;
            end
            
            if strcmp(type,'hyper') == 1
%                 self.decay = @(r) 0.5*(1-tanh(4*r/self.decayR-4));
%                 self.decay = @(r) 0.5*(1-tanh(6*r-4));

                   
                %tan hyper decay with max strength = 1
%                 self.decay = @(r) -(tanh(2*pi*r/self.decayR-pi)+1)/2+1;

            self.decay = @(r) -(tanh(2*pi*r/self.decayR-pi))+1;


            end
            
        end
        
        
        % ====================== Plotting Functions ==============================%
        function pltff(self)
            [X,Y,U,V] = self.ffLocal();
            quiver(X,Y,U,V,'r');
        end
        
        
        function pltfnc(self)
            theta = 0:0.01:2*pi;
            cxs = self.x+self.r*cos(theta);
            cys = self.y+self.r*sin(theta);
            plot(cxs,cys,'r',self.x,self.y,'rx','linewidth',2);
        end
        
  
        function plt = pltDecay(self)
            theta = 0:0.05:2*pi;
            cxs = self.x+self.decayR*cos(theta);
            cys = self.y+self.decayR*sin(theta);
            plt = plot(cxs,cys,'k--',self.x,self.y,'r*','linewidth',2.5);
            
        end
        
        function plt = pltEqualStrength(self)
            
            syms radius
            equalStrengthRadius = solve(self.decay(radius)==1,radius);
            
            theta = 0:0.05:2*pi;
            cxs = self.x+equalStrengthRadius*cos(theta);
            cys = self.y+equalStrengthRadius*sin(theta);
            plt = plot(cxs,cys,'r--',self.x,self.y,'r*','linewidth',2.5);  
        end
        
     
        
    end
end
