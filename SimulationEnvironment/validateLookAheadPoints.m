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
vf.avf{1}.r = 25;
vf.avf{1}.H = 1;
% vf.avf{1}.angle = pi/2;


dt = 0.1;

t = 1;          %Lookahead time

x = -10;
y = 0;
v = 5;


uav = uav.setup(x,y,v,0,dt);

vf.avf{1}.normComponents = 0;
vf.avf{1}.normTotal = 0;
vf.avf{1}.type = 'channel';
vf.avf{1}.e = 20;

vf = vf.xydomain(50,0,0,50);
vf.normAttractiveFields = 1;
vf.NormSummedFields = 0;

figure

for i =1:length(T)
    
    [U,V] = vf.heading(uav.x,uav.y);
    heading = atan2(V,U);
    uav = uav.update_pos(heading);
    
    
    hold on
    uav.pltUAV;
    axis equal
    vf.pltff

    drawnow
    clf
    
end


 
 
 
 
 










