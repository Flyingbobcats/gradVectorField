%==========================================================================
%
% costGammaCirculation.m
% Determine if performance cost is fixed for a given ratio:
%
%               Decay Radius
%     gamma =  --------------
%              turning radius
%
%
%
%  Vary gamma, velocity, and circulation for cost performance
%==========================================================================

clc
clear
close all



plotIndividualScenarioRealTime = false;
plotIndividualScenarioCompleted = false;
plotGammaCost = false;

%Obstacle Circulations
obstCircs = 1:.1:10;

%Radii ratios to evaluate performance at
gammas = .1:.1:5;

%Cost array for all simulations
COSTS = [];

%Velocities to evaluate performance at
velocity = 50;
vs = 50;



for k = 1:length(obstCircs)
    for i = 1:length(gammas)
        gamma = gammas(i);
        h = obstCircs(k);
        
        %Setup vector field
        vf = vectorField();
        
        %Goal Path
        vf = vf.navf('line');
        vf.avf{1}.angle = pi/2;
        vf.NormSummedFields = 0;
        vf.avf{1}.H = 2*velocity;%*velocity;
        vf.avf{1}.normComponents = false;
        vf.normAttractiveFields = false;
        
        %Obstacle
        vf = vf.nrvf('circ');
        vf.rvf{1}.r = 0.01;
        vf.rvf{1}.H = h;
        vf.rvf{1}.G = -1;
        vf.rvf{1}.y = -100;
        
        %Obstacle (no fly zone radius)
        obstR = velocity/0.35;
        obstx = obstR*cos(0:0.1:2.1*pi)+vf.rvf{1}.x;
        obsty = obstR*sin(0:0.1:2.1*pi)+vf.rvf{1}.y;
        
        
        
        %Setup UAV initial position (x will be changed later)
        xs =  -(velocity/0.35*gamma+obstR)*1.1;
        ys = 0;
        heading = 0;
        dt = .1;
        
        %Create UAV class instance
        uav = UAV();
        uav = uav.setup(xs,ys,velocity,heading,dt);
        
        %UAV plot settings
        uav.plotHeading = false;
        uav.plotCmdHeading = false;
        uav.plotUAV = false;
        uav.plotUAVPath = true;
        uav.plotFlightEnv = false;
        
        
        
        
        %Change UAVs starting position to just outside the decay field edge
        
        
        %Set decay field strength based on gamma ratio, plus obstacles
        %radius
        
        vf.rvf{1}.decayR = uav.turn_radius*gamma+obstR;
        vf.rvf{1} = vf.rvf{1}.modDecay('hyper');
        
        %Cost function initially 0
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
                cost = cost+dt;
            end
            if plotIndividualScenarioRealTime
                clf
                hold on
                vf.rvf{1}.pltDecay();
                plot(obstx,obsty,'b');
                uav.pltUAV();
%                 str = strcat('velocity = ',num2str(vs(j)), ' cost= ',num2str(cost),'\gamma=',num2str(gamma),'h=',num2str(h));
%                 title(str);
                axis equal
                pause(0.001)
            end
        end
        
        
        if plotIndividualScenarioCompleted
            hold on
            uav.colorMarker='-';
            uav.pltUAV();
            xlabel('x');
            ylabel('y');
            str = strcat('v=',num2str(velocity),'\gamma = ', num2str(gamma),'h=',num2str(h));
            title(str);
            axis equal
            drawnow
        end
        
        COSTS(i,k) = cost;
        VELOCITIES(i,k) = velocity;
        GAMMAS(i,k) = gamma;
        HS(i,k) = h;
        
        if plotGammaCost
            hold on
            plot(gamma,cost,'k.');
            xlabel('\gamma');
            ylabel('cost');
            grid on
        end
    end
    clc
    str = strcat(num2str((k)/(length(obstCircs))*100), '%', {' '}, 'complete');
    disp(str);
end


figure
surf(GAMMAS,HS,COSTS);
xlabel('\gammas');
ylabel('h');
zlabel('cost');











