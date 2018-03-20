% 
%                                            _______
% UAV traveling on a 'pulse' path     ____  |       |____
%
%
clc
clear
close all


% v = VideoWriter('pulse');
% open(v)
 
t0 = 0;
tf = 300;
dt = .1;

t = t0:dt:tf;

vf = vectorField;
vf = vf.navf('circ');
vf.avf{1}.r = (1/0.35)*1.5;


uav = UAV;
uav = uav.setup(-10,0,1,0,dt);

hold on


Vel = 1/5;
pause()

for i=1:length(t)
   
    path_heading = loadPath('pulse',t(i));
    
    uav = uav.update_pos(vf);
    vf.avf{1}.x = vf.avf{1}.x+Vel*cos(path_heading)*dt;
    vf.avf{1}.y = vf.avf{1}.y+Vel*sin(path_heading)*dt;
    
    vf.avf{1}.vx = Vel*cos(path_heading);
    vf.avf{1}.vy = Vel*sin(path_heading);
    
    vf = vf.xydomain(10,vf.avf{1}.x,vf.avf{1}.y,30);
    

    clf
    hold on
    
    %     vf.pltff
    vf.avf{1}.pltfnc
    uav.pltUAV
    
    axis equal
    axis([-30,30,-20,20]);
    grid on
    drawnow
    
%     writeVideo(v,getframe(gcf));
    
end

% close(v);





