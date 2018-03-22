%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Observe the tracking performance of a UAV loitering about a moving point.
% A UAVs minimum turn radius:
%
%                 velocity      (m/s)
%    radius =    ----------
%                 turnrate     (rad/s)
%
%   Findings:
%
%   If the loiter radius < UAV minimum radius, the tracking error is
%   significant and oscilates
%
%   If the loiter radius = UAV minimum turn radius, the tracking error is
%   small and has small oscilations
%
%   If the loiter radius = 1.5 * UAV minimum radius, the tracking error is
%   near zero with small oscilations
%
%
%

clc
clear
close all

UAV_velocity = 5;
V_ratio = (1/5);
turn_rate = 0.35;
V_target = V_ratio*UAV_velocity;


%Create array of ratios
R_ratios = linspace(1,2,40);

for j = 1:length(R_ratios)
    error = [];
    figure
    
    %Simulation time setup
    dt = 0.1;
    tf=200;
    t = 0:dt:tf;
    
    %Initialize vector field object, add attractive field, setup properties
    vf = vectorField;
    vf = vf.navf('circ');
    vf.avf{1}.vx = 1;
    vf.avf{1}.H = 1;
    vf.avf{1}.G = 1;
    vf.avf{1}.L = 1;
    vf = vf.xydomain(50,0,0,50);
    
    %Calculate the loiter radius
    R_loiter = (UAV_velocity / turn_rate) * R_ratios(j);
    
    %Set loiter radius
    vf.avf{1}.r = R_loiter;
    
    %Initalize UAV
    uav = UAV;
    uav.turnrate = turn_rate;
    uav.x = -35-vf.avf{1}.r;
    uav.y = 0;
    uav.v = UAV_velocity;
    uav.vx = UAV_velocity;
    uav.vy = 0;
    uav.heading = 0;
    
    %Run simulation
    for i =1:length(t)
        [u,v] = vf.heading(uav);
        heading = atan2(v,u);
        vf.avf{1}.x = vf.avf{1}.x+V_target*dt;
        error(i) = sqrt((uav.x-vf.avf{1}.x)^2+(uav.y-vf.avf{1}.y)^2)-vf.avf{1}.r;
        uav = uav.update_pos(heading);
    end
    
    %Find peak error and index
    [peakError,index] = max(error(300:end));
    peakErrors(j) = peakError;
    
    %Plot the UAV path and the error
    subplot(2,1,1);
    uav.pltUAV
    vf.avf{1}.pltfnc
    plot(uav.xs,uav.ys,'.k');
    xlabel('East');
    ylabel('North');
    axis([-50,200,-40,40]);
    str = strcat('Loiter Radius Ratio = ',num2str(R_ratios(j)));
    title(str)
    grid on
    axis equal
    
    subplot(2,1,2)
    hold on
    plot(t,error);
    plot(t(299+index),peakError,'r*')
    xlabel('time');
    ylabel('error');
    grid on
    
    
end

%Plot the errors versus loiter radius
figure
plot(R_ratios,peakErrors,'ok');
grid on
xlabel('Radius Ratios');
ylabel('Peak Error');

