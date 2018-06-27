%==========================================================================
% validateLookAheadPoints.m
%
%
%==========================================================================

clc
clear
close all


dt = 0.01;
ts = 0;
tf = 5;

T = ts:dt:tf;

uav = UAV();
vf = vectorField();
vf = vf.navf('circ');
vf.avf{1}.r = 10;
vf.avf{1}.H = 1;
vf.avf{1}.G = 1;

dt = 0.1;
t = 1;          %Lookahead time

x = -10;
y = 0;
v = 5;


uav = uav.setup(x,y,v,0,dt);

vf.avf{1}.normComponents = 1;
vf.avf{1}.normTotal = 1;

vf.avf{1}.decayActive = 0;

vf = vf.xydomain(50,0,0,50);
vf.normAttractiveFields = 0;
vf.NormSummedFields = 0;

figure

for i =1:length(T)
    
    [U,V] = vf.heading(uav.x,uav.y);
    heading = atan2(V,U);
    uav = uav.update_pos(heading);
    
    
    hold on
    uav.pltUAV;
    axis equal
    vf.pltff;

    drawnow
    clf
    
end


 
 
 
 
 










