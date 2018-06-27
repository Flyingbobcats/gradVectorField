%==========================================================================
% getOnPathMultpleHs.m
% Description:
% Script demonstrates a Dubin's UAV guided by a path following VF guidance.
% Multiple circulation values (H) are provided and the flight path
% observed. Two figures are produced (1) Flight path and (2) Lateral error
% from the desired target path.
%
%
%                                                    Author: Garrett Clem
%==========================================================================


clc
clear
close all

%Obstacle present
obstacle = false;

%UAV initial conditions
xs = -150;
ys = 50;
heading = 0;
dt = 0.1;
velocity = 20;



lateralDistance = {};
Hs = [1,25,50];
hold on
plot([-200,200],[0,0],'g','linewidth',3);
plot(xs,ys,'db','markersize',10,'markerfacecolor','b');


for i = 1:length(Hs)
    
    %Instance of vector field class
    vf = vectorField();
    
    %Goal Path
    vf = vf.navf('line');
    vf.avf{1}.angle = pi/2;
    vf.NormSummedFields = false;
    vf.avf{1}.H = Hs(i);
    vf.avf{1}.normComponents = false;
    vf.normAttractiveFields = false;
    vf = vf.xydomain(150,0,0,30);
    
    
    %If obstacle present, add to VF
    if obstacle
        vf = vf.nrvf('circ');
        vf.rvf{1}.r = 0.01;
        vf.rvf{1}.H = 0;
        vf.rvf{1}.G = -1;
        vf.rvf{1}.y = 0;
        vf.rvf{1}.decayR = 119;
    end
    
    %Create UAV class instance
    uav = UAV();
    uav = uav.setup(xs,ys,velocity,heading,dt);
    
    %UAV plot settings
    uav.plotHeading = false;
    uav.plotCmdHeading = false;
    uav.plotUAV = false;
    uav.plotUAVPath = true;
    uav.plotFlightEnv = false;
    
    latDist = [];
    time = [];
    
    %Simulation starts here
    while uav.x<150
        [u,v]=vf.heading(uav.x,uav.y);
        heading_cmd = atan2(v,u);
        uav = uav.update_pos(heading_cmd);
        latDist = [latDist,uav.y];
        time = [time,uav.t];
        
        
    end
    
    lateralDistance{i} = [latDist;time]';
    
    hold on
    set(gca,'fontsize',12);
    plot(uav.xs,uav.ys,'linewidth',2);
    xlabel('x');
    ylabel('y');   
    axis equal
    axis([-200,200,-50,150]);
    xticks(-200:50:300);
end

names{1} = 'Planned Path';
names{2} = 'UAV Start';
for j=1:length(Hs)
    names{j+2} = strcat('H = ',num2str(Hs(j))); 
end


plot([148.4,150],[0,-25],'sr','markersize',10,'markerfacecolor','r');
names{end+1} = 'UAV End';
leg = legend(names);


%Plot lateral error
figure
for i =1:length(lateralDistance)
    hold on
    plot(lateralDistance{i}(:,2),lateralDistance{i}(:,1),'linewidth',2);
    set(gca,'fontsize',12);
    xlabel('time [s]');
    ylabel('Lateral Error [m]');
    axis([0,15,-50,50]);
    grid on
    names{i} = strcat('H=',num2str(Hs(i)));
end

legend(names);








