%=========================================================================
% numericallySolveGammaAndH.m
%
%
%==========================================================================



clc
clear
close all

v = 50;
dt = 0.1;

plotFinal = false;

fr = @(X) VF(X,v,dt,plotFinal);         %Find min with respect to r



options = optimoptions('fmincon','Display','final-detailed');
options.DiffMinChange = 0.01;
options.DiffMaxChange = 0.05;
options.PlotFcn = @optimplotfval;
options.StepTolerance = 1e-8;

x0 = [1,1];
A = [];
b = [];
Aeq = [];
beq = [];
lb = [1,0.1];
ub = [4,6];



tic
[Xsolved,costR] = fmincon(fr,x0,A,b,Aeq,beq,lb,ub,[],options);
toc


plotFinal = true;
fr = @(X) VF(X,v,dt,plotFinal);
fr(Xsolved);
disp(Xsolved)





function cost = VF(X,velocity,dt,plotFinal)

    gamma = X(1);
    H = X(2);

    %Setup vector field
        vf = vectorField();
        
        vf = vf.xydomain(200,0,0,50);
        
        %Goal Path
        vf = vf.navf('line');
        vf.avf{1}.angle = pi/2;
        vf.NormSummedFields = false;
        vf.avf{1}.H = 2*velocity;%*velocity;
        vf.avf{1}.normComponents = false;
        vf.normAttractiveFields = false;

        
        %Obstacle
        vf = vf.nrvf('circ');
        vf.rvf{1}.r = 0.01;
        vf.rvf{1}.H = H;
        vf.rvf{1}.G = -1;
        vf.rvf{1}.y = 0;
        
        %Obstacle (no fly zone radius)
        obstR = velocity/0.35;
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
        uav.plotFlightEnv = true;
        
        

        
        %Change UAVs starting position to just outside the decay field edge

        
        %Set decay field strength based on gamma ratio, plus obstacles
        %radius
        
        vf.rvf{1}.decayR = uav.turn_radius*gamma+obstR;
        
        vf.rvf{1} = vf.rvf{1}.modDecay('hyper');
        
        %Cost function initially 0
        cost = 0;

    cost = 0;
    while uav.x<=(uav.turn_radius*gamma+obstR)*1.1

        [u,v]=vf.heading(uav.x,uav.y);
        heading_cmd = atan2(v,u);

        if uav.heading > deg2rad(175) && uav.heading <deg2rad(285)
            cost = 10;
            break
        end
        uav = uav.update_pos(heading_cmd);


        %Deviation from path cost, normalized with respect to obstacles
        %radius
        cost = cost+ abs(uav.y) / ((vf.rvf{1}.decayR))*dt;

        range = sqrt(uav.x^2+uav.y^2);
        if range < obstR
            cost = cost+100;
        end

        if plotFinal == true
            clf
            hold on
            vf.rvf{1}.pltDecay();
            plot(obstx,obsty,'b');
            uav.pltUAV();
            %                 str = strcat('velocity = ',num2str(vs(j)), ' cost= ',num2str(cost),'\gamma=',num2str(gamma),'h=',num2str(h));
            %                 title(str);
            axis equal
            pause(0.001)
            cost
            end
    end

end











