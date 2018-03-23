%==========================================================================
%
% costStraightPathFollowing.m
%
% Determine if following a straight path and oscillations due to Dubins
% vehicle will have significant cost
%
%
%==========================================================================

clc
clear
close all

vf = vectorField();

%Goal Path
vf = vf.navf('line');
vf.avf{1}.angle = pi/2;
vf.NormSummedFields = 0;
vf.avf{1}.normComponents = false;
vf.normAttractiveFields = false;

vf.pltff();

xs = -10;
ys = 0;
heading = 0;
v=1;
dt = 0.1;

uav = UAV();
uav = uav.setup(xs,ys,v,heading,dt);

cost = 0;
while uav.x<10
    
    [u,v]=vf.heading(uav.x,uav.y);
    heading_cmd = atan2(v,u);
    uav = uav.update_pos(heading_cmd);
    cost = cost+uav.y;
end

hold on
uav.pltUAV();
disp(cost);
