%=========================================================================
% numericallySolveGammaAndH.m
%
%
%==========================================================================



clc
clear
close all
ns = 1:3:10;
vs = 10:20:100;

NS = [];
VS = [];
COSTS = [];
XSOLVED = {};



for k = 1:length(ns)
for j=1:length(vs)
    figure
    n = ns(k);
    v = vs(j);

obstR = n*v/0.35;
obstY = 0;
tr = v/0.35;
dt = 0.01;

plotFinal = false;

fr = @(X) VF(X,v,dt,plotFinal,obstR,obstY,n);         %Find min with respect to r



options = optimoptions('fmincon','Display','off');
options.DiffMinChange = 0.1;
options.DiffMaxChange = 0.2;
% options.PlotFcn = @optimplotfval;
options.StepTolerance = 1e-5;

x0 = [3,2.8];
A = [];
b = [];
Aeq = [];
beq = [];
lb = [2.5,2.5];
ub = [3.5,4];



tic
% [Xsolved,costR] = fmincon(fr,x0,A,b,Aeq,beq,lb,ub,[],options);
sim_time = toc;

Xsolved = [2.5,3];


plotFinal = true;
fr = @(X) VF(X,v,dt,plotFinal,obstR,obstY,n);
cost = fr(Xsolved);
% disp(Xsolved)
% disp(cost)
str = strcat('h = ',num2str(Xsolved(1)),{' '}, '\gamma=',num2str(2*n),{' '},'cost=',num2str(cost),{' '},'time=',num2str(sim_time),'k=',num2str(Xsolved(2)));
title(str)

NS(k,j) = ns(k);
VS(k,j) = vs(j);
COSTS(k,j) = cost;
XSOLVED{k,j} = Xsolved;


end
    clc
    str = strcat('Percent complete:', num2str((k)/(length(ns))*100));
    disp(str);
end





function cost = VF(X,velocity,dt,plotFinal,obstR,obstY,n)

    k = X(2);
    H = X(1);
    gamma = 2*n;

    %Setup vector field
        vf = vectorField();
        
        vf = vf.xydomain(200,0,0,50);
        
        %Goal Path
        vf = vf.navf('line');
        vf.avf{1}.angle = pi/2;
        vf.NormSummedFields = false;
%         vf.avf{1}.H = 2*H*velocity^1/2;%*velocity;
        vf.avf{1}.H = obstR;
        vf.avf{1}.normComponents = false;
        vf.normAttractiveFields = false;

        
        %Obstacle
        vf = vf.nrvf('circ');
        vf.rvf{1}.r = 0.01;
        vf.rvf{1}.H = H;
        vf.rvf{1}.G = -1;
        vf.rvf{1}.y = obstY;
        
        %Obstacle (no fly zone radius)
%         obstR = velocity/0.35+150;
        obstx = obstR*cos(0:0.1:2.1*pi)+vf.rvf{1}.x;
        obsty = obstR*sin(0:0.1:2.1*pi)+vf.rvf{1}.y;
        
        
        
        %Setup UAV initial position (x will be changed later)
        xs =  -(velocity/0.35*gamma+obstR)*1.1;
        ys = 0;
        heading = 0;
  
        
        %Create UAV class instance
        uav = UAV();
        uav = uav.setup(xs,ys,velocity,heading,dt);
        
        %UAV plot settings
        uav.plotHeading = false;
        uav.plotCmdHeading = false;
        uav.plotUAV = false;
        uav.plotUAVPath = true;
        uav.plotFlightEnv = false;
        uav.colorMarker = 'k--';
        
        

        
        %Change UAVs starting position to just outside the decay field edge

        
        %Set decay field strength based on gamma ratio, plus obstacles
        %radius
        
%         vf.rvf{1}.decayR = uav.turn_radius*gamma+obstR;
%         vf.rvf{1}.decayR = uav.turn_radius*(3*n);
        vf.rvf{1}.decayR = uav.turn_radius*(k*n);
        
        vf.rvf{1} = vf.rvf{1}.modDecay('hyper');
        
        %Cost function initially 0
        cost = 0;

    cost = 0;
    while uav.x<=(uav.turn_radius*gamma+obstR)*1.1

        [u,v]=vf.heading(uav.x,uav.y);
        heading_cmd = atan2(v,u);

        if uav.heading > deg2rad(175) && uav.heading <deg2rad(285)
            cost = 1000;
            break
        end
        uav = uav.update_pos(heading_cmd);


        %Deviation from path cost, normalized with respect to obstacles
        %radius
        cost = cost+ abs(uav.y) / ((vf.rvf{1}.decayR))*dt;

        range = sqrt((uav.x-vf.rvf{1}.x)^2+(uav.y-vf.rvf{1}.y)^2);
        if range < obstR
            cost = cost+100;
        end

      
    end
      if plotFinal == true
            clf
            hold on

            plot(obstx,obsty,'r','linewidth',2);
            plot(uav.xs(1),uav.ys(1),'d','markersize',10,'markerfacecolor','b');
            plot([-250,250],[0,0],'g','linewidth',2);
            uav.pltUAV();
            
            xlabel('x');
            ylabel('y');
            set(gca,'fontsize',12);

            axis equal
            grid on

            
            
            legend({'Obstacle','UAV Start','Pre-planned Path','UAV Path'});
            end
    

end











