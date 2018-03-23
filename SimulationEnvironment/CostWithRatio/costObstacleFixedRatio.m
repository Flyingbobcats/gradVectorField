%==========================================================================
%
% costObstacleFixedRatio.m
% Determine if performance cost is fixed for a given ratio:
%
%               Decay Radius
%     gamma =  --------------
%              turning radius
%
%
%
% a range of gamma ratios are evaluated. Each gamma is evaluated at
% multiple velocities. All other parameters are fixed
%==========================================================================

clc
clear
close all



plotIndividualScenarioRealTime = false;
plotIndividualScenarioCompleted = false;
plotGammaCost = true;

%Cost array for all simulations
COSTS = [];

%Velocities to evaluate performance at
vs = 10:20:100;
% vs = 50;


%Radii ratios to evaluate performance at
gammas = 1:.1:5;
% gammas = 1.9999;


for i = 1:length(gammas)
    for j=1:length(vs)
        
        gamma = gammas(i);
        velocity = vs(j);
        
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
        vf.rvf{1}.H = 2;
        vf.rvf{1}.G = -1;
        vf.rvf{1}.y = 0;
        
        %Obstacle (no fly zone radius)
        obstR = velocity/0.35;
%         obstR = 5;
        obstx = obstR*cos(0:0.1:2.1*pi)+vf.rvf{1}.x;
        obsty = obstR*sin(0:0.1:2.1*pi)+vf.rvf{1}.y;
        
        
        
        %Setup UAV initial position (x will be changed later)
        xs =  -(velocity/0.35*gamma+obstR)*1.1;
        ys = 0;
        heading = 0;
        dt = .01;
        
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

        while uav.x<=(uav.turn_radius*gamma+obstR)*1.1
            
%             los = atan2(-uav.y,-uav.x)
%             strength = 2/(abs(los)+1)
%             vf.rvf{1}.H = strength;
            [u,v]=vf.heading(uav.x,uav.y);
            heading_cmd = atan2(v,u);
            uav = uav.update_pos(heading_cmd);
            
     
            %Deviation from path cost, normalized with respect to obstacles
            %radius
            cost = cost+ uav.y / ((vf.rvf{1}.decayR))*dt;

            if plotIndividualScenarioRealTime
                clf
                hold on
%                 vf.pltff
                vf.rvf{1}.pltDecay();
                plot(obstx,obsty,'b');
                uav.pltUAV();
                str = strcat('velocity = ',num2str(vs(i)), ' cost= ',num2str(cost));
                title(str);
                axis equal
                drawnow
            end    
        end

        
        if plotIndividualScenarioCompleted
            hold on
            uav.colorMarker='-';
            uav.pltUAV();
            xlabel('x');
            ylabel('y');
            str = strcat('\gamma = ', num2str(gamma));
            title(str);
            axis equal
            drawnow
        end



        
        COSTS(i,j) = cost;
        VELOCITIES(i,j) = velocity;
        GAMMAS(i,j) = gamma;
        
        if plotGammaCost
            hold on
            plot(gamma,cost,'k.');
            xlabel('\gamma');
            ylabel('cost');
            grid on
        end
        
    end
   
   clc 
    str = strcat(num2str((i)/(length(gammas))*100), '%', {' '}, 'complete');
    disp(str);
end













