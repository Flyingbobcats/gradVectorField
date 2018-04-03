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
obstR = v/0.35;
obstY = 0;
tr = v/0.35;
lbGamma = (obstR / (tr));
dt = 0.1;

plotFinal = false;
fr = @(X) VF(X,v,dt,plotFinal,obstR,obstY);         %Find min with respect to r
options = optimoptions('fmincon','Display','final-detailed');
options.DiffMinChange = 0.5;
options.DiffMaxChange = 1;
options.PlotFcn = @optimplotfval;
options.StepTolerance = 1e-4;

x0 = [lbGamma]; 
A = [];
b = [];
Aeq = [];
beq = [];
lb = [lbGamma*0.5];
ub = [lbGamma*3];

tic
[Xsolved,costR] = fmincon(fr,x0,A,b,Aeq,beq,lb,ub,[],options);
sim_time = toc;


plotFinal = true;
figure
pause()
fr = @(X) VF(X,v,dt,plotFinal,obstR,obstY);
cost = fr(Xsolved);
% disp(Xsolved)
% disp(cost)
% str = strcat('\gamma = ',num2str(Xsolved(1)),{' '}, 'h=',num2str(Xsolved(2)),{' '},'cost=',num2str(cost),{' '},'time=',num2str(sim_time));
% % title(str)
% xlabel('x');
% ylabel('y');



function cost = VF(X,velocity,dt,plotFinal,obstR,obstY)

    gamma = X(1);
%     H = X(2);


    %Setup vector field
        vf = vectorField();
        
        vf = vf.xydomain(3*obstR,0,0,20);
        
        %Goal Path
        vf = vf.navf('line');
        vf.avf{1}.angle = pi/2;
        vf.NormSummedFields = true;
        vf.avf{1}.H = obstR;

        vf.avf{1}.normComponents = false;
        vf.normAttractiveFields = false;

        
        %Obstacle
        vf = vf.nrvf('circ');
        vf.rvf{1}.r = 0.01;
        vf.rvf{1}.H = 1;
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
    
        TH = [];
        BETA = [];
        ALPHA = [];
        P = [];
        hs = [];
        gs = [];
    
    while uav.x<=(uav.turn_radius*gamma+obstR)*1.1
        
        
        [u,v]=vf.heading(uav.x,uav.y);
        heading_cmd = atan2(v,u);
        uav = uav.update_pos(heading_cmd);

        
        
        range = sqrt((uav.x-vf.rvf{1}.x)^2+(uav.y-vf.rvf{1}.y)^2);
        
        if uav.heading > deg2rad(175) && uav.heading <deg2rad(285)
            cost = 100000;
            break
        end
     
        if abs(uav.x)>=obstR
            cost = cost + abs(uav.y)/obstR;

            
        else
            cost = cost + abs((range-obstR))/obstR;

        end
        
        if range < obstR
            cost = cost+100;
        end

        
        
        th = uav.heading;
        beta =  atan2(-uav.y,-uav.x);
        alpha = th+beta;
        
        TH = [TH, th];
        BETA = [BETA,beta];
        ALPHA = [ALPHA,alpha];
        
        
        
         p = abs(-(tanh(2*pi*abs(beta)/deg2rad(270)-pi))+1);
         
         P = [P,p];
         vf.rvf{1}.G = -p/2;
         vf.rvf{1}.H = 2/p;
         vf.avf{1}.H = 2*uav.v;
         
         
         
         
         hs = [hs,vf.rvf{1}.H];
         gs = [gs,vf.rvf{1}.G];

        if plotFinal == true
            [dist,location] = calcLatDistance(uav.x,uav.y,optPath);
   
            clf
            
            subplot(3,2,[2,4,6]);
            hold on
            plot([uav.x,optPath(location(1),1)],[uav.y,optPath(location(1),2)],'g');
            plot(optPath(:,1),optPath(:,2),'k.');
            vf.rvf{1}.pltDecay();
            plot(obstx,obsty,'b');
            uav.pltUAV();
            title(num2str(cost));
            axis equal
            
            subplot(3,2,1);
            plot(rad2deg(BETA));
            ylabel('decay');
            grid on
            
            
            subplot(3,2,3);
            plot(gs);
            ylabel('g');
            grid on
            
            subplot(3,2,5);
            plot(hs);
            ylabel('h');
            grid on
                        
            
            drawnow()
                        

        end
    end
    
    if plotFinal == true
        
%         figure
%         hold on
%         plot(rad2deg(TH));
%         plot(rad2deg(BETA))
%         plot(rad2deg(ALPHA));
%         legend({'theta','beta','alpha'});
    end

end
