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
vs = 1:.3:20;

%Radii ratios to evaluate performance at
gammas = 2:1:2;

for i = 1:length(gammas)
%     figure
    for j=1:length(vs)
              
        gamma = gammas(i);
        velocity = vs(j);
        
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
        vf.rvf{1}.r = 0.1;
        vf.rvf{1}.H = 2;
        vf.rvf{1}.G = -1;
        
        xs = 0;
        ys = 0;
        heading = 0;
        dt = .1;
        
        uav = UAV();
        uav = uav.setup(xs,ys,velocity,heading,dt);
        
        uav.plotHeading = false;
        uav.plotCmdHeading = false;
        
        uav.plotUAV = false;
        uav.plotUAVPath = true;
        uav.plotFlightEnv = false;
        
        
        
        obstR = uav.turn_radius;
        obstx = obstR*cos(0:0.1:2*pi);
        obsty = obstR*sin(0:0.1:2*pi);
        
        uav.x = -(uav.turn_radius*gamma+obstR+uav.v);
        
        
        vf.rvf{1}.decayR = uav.turn_radius*gamma+obstR;
        vf.rvf{1} = vf.rvf{1}.modDecay('hyper');
        
        cost = 0;
        
        while uav.x<=2*uav.turn_radius*gamma
            [u,v]=vf.heading(uav.x,uav.y);
            heading_cmd = atan2(v,u);
            uav = uav.update_pos(heading_cmd);
            
% %             %Tracking outside of obstacle cost
%                 range = sqrt(uav.x^2+uav.y^2);
%                 if range<=vf.rvf{1}.decayR
%                     cost = cost+abs((range - uav.turn_radius))/(uav.turn_radius*gamma+obstR)*dt;
%                 else
%                     cost = cost+0;
%                 end
%             
%             
%           Deviation from path
            
            if abs(uav.y)>uav.v*dt
            cost = cost+ uav.y / ((uav.turn_radius*gamma+obstR))*dt;
            end
            
          
%                 clf
%                 hold on
%                 vf.rvf{1}.pltDecay();
%                 plot(obstx,obsty,'b');
% %                 vf.rvf{1}.pltEqualStrength();
%                 uav.pltUAV();
% %                 plot([-2*(uav.turn_radius*gamma+obstR*1.1),(uav.turn_radius*gamma+obstR*1.1)*2],[0,0],'g--','linewidth',2);
%                 str = strcat(num2str(vs(i)), ' cost= ',num2str(cost));
%                 title(str);
%                 axis equal
%                 drawnow
       
            
            
        end
        
                hold on
%                 vf.rvf{1}.pltDecay();
                plot(obstx,obsty,'b');
%                 vf.rvf{1}.pltEqualStrength();
                uav.pltUAV();
%                 plot([-2*(uav.turn_radius*gamma+obstR*1.1),(uav.turn_radius*gamma+obstR*1.1)*2],[0,0],'g--','linewidth',2);
                str = strcat(num2str(vs(i)), ' cost= ',num2str(cost));
                title(str);
                axis equal
                drawnow
        
        
        
        
%             hold on
%             uav.colorMarker='-';
%             uav.pltUAV(); 
%             set(gca,'fontsize',16);
%             xlabel('x');
%             ylabel('y');
%             title(strcat('\gamma=',num2str(gammas(i))));
%             grid on
%             axis equal
%             drawnow

        
        COSTS(i,j) = cost;
        VELOCITIES(i,j) = velocity;
        GAMMAS(i,j) = gamma;

    end
    for k=1:length(vs)   
    str = num2str(vs(k));
    legVel{k} = strcat('v = ',str);
    end
%     legend(legVel);
    clc
    
    str = strcat(num2str((i)/(length(gammas))*100), '%', {' '}, 'complete');
    disp(str);
end

figure
plot(vs,COSTS)




for i=1:length(gammas)   
    str = num2str(gammas(i));
    legGammas{i} = strcat('\gamma = ',str);
end

% figure
% plot(vs,COSTS)
% legend(legGammas)
% grid on
% xlabel('velocity');
% ylabel('path deviation cost');
% 
% 
% figure
% surf(VELOCITIES,GAMMAS,COSTS);
% xlabel('Velocity');
% ylabel('^{decayR}/_{turning_r}')
% zlabel('Path Deviation Cost');
% shading interp
% 
% figure
% plot(gammas,COSTS);
% xlabel('\gamma')
% ylabel('cost');
% legend(legVel)
% grid on







