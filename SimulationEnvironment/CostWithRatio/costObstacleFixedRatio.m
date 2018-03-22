%==========================================================================
%
% costObstacleFixedRatio.m
%
% Determine if performance cost is fixed for a given ratio:
%
%               Decay Radius
%     gamma =  --------------
%              turning radius
%==========================================================================

clc
clear
close all


%Cost array for all simulations
COSTS = [];

%Velocities to evaluate performance at
vs = 1:1:5;

%Radii ratios to evaluate performance at
gammas = 1:1:3;

for i = 1:length(gammas)
    for j=1:length(vs)
                figure
        gamma = gammas(i);
        velocity = vs(j);
        
        
        %Setup vector field
        vf = vectorField();
        
        %Goal Path
        vf = vf.navf('line');
        vf.avf{1}.angle = pi/2;
        vf.NormSummedFields = 0;
        vf.avf{1}.H = 2*velocity;
        vf.avf{1}.normComponents = false;
        vf.normAttractiveFields = false;
        
        %Obstacle
        vf = vf.nrvf('circ');
        vf.rvf{1}.r = 0.1;
        vf.rvf{1}.H = 2;
        vf.rvf{1}.G = -1;
        
        xs = -10*velocity;
        ys = 0;
        heading = 0;
        dt = .1;
        
        uav = UAV();
        uav = uav.setup(xs,ys,velocity,heading,dt);
        
        obstR = uav.turn_radius;
        obstx = obstR*cos(0:0.1:2*pi);
        obsty = obstR*sin(0:0.1:2*pi);
        
        
        vf.rvf{1}.decayR = uav.turn_radius*gamma+obstR;
        vf.rvf{1} = vf.rvf{1}.modDecay('hyper');
        
        cost = 0;
        
        while uav.x<10*velocity
            
            [u,v]=vf.heading(uav.x,uav.y);
            heading_cmd = atan2(v,u);
            uav = uav.update_pos(heading_cmd);
            
            %Tracking outside of obstacle cost
                range = sqrt(uav.x^2+uav.y^2);
                if range<=vf.rvf{1}.decayR
                    cost = cost+abs((range - uav.turn_radius));
                else
                    cost = cost+0;
                end
            
            
            %Deviation from path
%             cost = cost+ uav.y*dt;
            
            
            
%                 clf
%                 hold on
%                 vf.rvf{1}.pltDecay();
%                 plot(obstx,obsty,'b');
%             %     vf.rvf{1}.pltEqualStrength();
%             %     vf.pltff();
%                 uav.pltUAV();
%                 str = strcat(num2str(vs(i)), ' cost= ',num2str(cost));
%                 title(str);
%                 axis equal
%                 drawnow
%             
            
            
        end
        
            hold on
            vf.rvf{1}.pltDecay();
            plot(obstx,obsty,'b');
            vf.rvf{1}.pltEqualStrength();vf.pltff();
            uav.pltUAV();
            str = strcat('velocity=',num2str(velocity), ' cost= ',num2str(cost),' gamma=',num2str(gamma));
            title(str);
            axis equal
            drawnow
        
        COSTS(i,j) = cost;
        VELOCITIES(i,j) = velocity;
        GAMMAS(i,j) = gamma;
        i
        j
    end
end


for i=1:length(gammas)
    
    str = num2str(gammas(i));
    leg{i} = strcat('\gamma = ',str);
end

figure
plot(vs,COSTS)
legend(leg)
grid on
xlabel('velocity');
ylabel('path deviation cost');


figure
surf(VELOCITIES,GAMMAS,COSTS);
xlabel('Velocity');
ylabel('^{decayR}/_{turning_r}')
zlabel('Path Deviation Cost');
shading interp

% end

% for i=1:length(gammas)
%
%     str = num2str(gammas(i));
%     leg{i} = str;
% end
%
% figure
% plot(vs,COSTS);
% legend(leg);
%
%
%
% figure
% plot(vs,t_rates);




