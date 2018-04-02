%========================================================================
% optPathAndVF.m
%
% Determine constant VF weight and decay radius for minimizing deviation
% from the geometric optimal path
%
% overlay optimal path and vector field guidance.
%
%=========================================================================

clc
clear
close all

v = 10;
obstR = v/0.35+100;
obstY = 0;
tr = v/0.35;
lbGamma = (obstR / (tr));
dt = 0.1;

plotFinal = false;
fr = @(X) VF(X,v,dt,plotFinal,obstR,obstY);         %Find min with respect to r
options = optimoptions('fmincon','Display','final-detailed');
options.DiffMinChange = 0.05;
options.DiffMaxChange = 0.5;
options.PlotFcn = @optimplotfval;
options.StepTolerance = 1e-4;

x0 = [lbGamma*1.5,2]; 
A = [];
b = [];
Aeq = [];
beq = [];
lb = [lbGamma*0.5,0.1];
ub = [lbGamma*2,6];

tic
[Xsolved,costR] = fmincon(fr,x0,A,b,Aeq,beq,lb,ub,[],options);
sim_time = toc;


plotFinal = true;
figure
pause()
fr = @(X) VF(X,v,dt,plotFinal,obstR,obstY);
cost = fr(Xsolved);
disp(Xsolved)
disp(cost)
str = strcat('\gamma = ',num2str(Xsolved(1)),{' '}, 'h=',num2str(Xsolved(2)),{' '},'cost=',num2str(cost),{' '},'time=',num2str(sim_time));
title(str)
xlabel('x');
ylabel('y');



function cost = VF(X,velocity,dt,plotFinal,obstR,obstY)

    gamma = X(1);
    H = X(2);


    %Setup vector field
        vf = vectorField();
        
        vf = vf.xydomain(200,0,0,50);
        
        %Goal Path
        vf = vf.navf('line');
        vf.avf{1}.angle = pi/2;
        vf.NormSummedFields = false;
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
%        obstR = velocity/0.35+150;
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
        uav.colorMarker = 'r.';
        
        optPath = genOptPath(uav,obstR);
        
        

        


        
        %Set decay field strength based on gamma ratio, plus obstacles
        %radius
     
        vf.rvf{1}.decayR = uav.turn_radius*gamma+obstR;
        vf.rvf{1} = vf.rvf{1}.modDecay('hyper');
        
        %Cost function initially 0
        cost = 0;

    cost = 0;
    while uav.x<=(uav.turn_radius*gamma+obstR)*1.1
        
          
%         LOS = atan2(-uav.y,-uav.x)-uav.heading;
%         rad2deg(LOS);
%         p = -1*(-(tanh(2*pi*LOS/(pi/2)-pi))+1);
%         
%         vf.rvf{1}.G = p;
%         
%         if uav.x>0
%             vf.rvf{1}.G = 0;
%         end

        [u,v]=vf.heading(uav.x,uav.y);
        heading_cmd = atan2(v,u);

        if uav.heading > deg2rad(175) && uav.heading <deg2rad(285)
            cost = 1000;
            break
        end
        uav = uav.update_pos(heading_cmd);


        %Deviation from path cost, normalized with respect to obstacles
        %radius
%         cost = cost+ abs(uav.y) / ((vf.rvf{1}.decayR))*dt;
         [dist,location] = calcLatDistance(uav.x,uav.y,optPath);
         cost = cost + dist;
        
         

        range = sqrt((uav.x-vf.rvf{1}.x)^2+(uav.y-vf.rvf{1}.y)^2);
        if range < obstR
            cost = cost+100;
        end

        if plotFinal == true
            
            [dist,location] = calcLatDistance(uav.x,uav.y,optPath);
   
            clf
            hold on
            plot([uav.x,optPath(location(1),1)],[uav.y,optPath(location(1),2)],'g');
            plot(optPath(:,1),optPath(:,2),'k.');
            vf.rvf{1}.pltDecay();
            plot(obstx,obsty,'b');
            uav.pltUAV();
            title(num2str(dist));
            axis equal
            drawnow()
            grid on
            end
    end
    
    if plotFinal == true
        vf.pltff()
    end

end
