% 
%
%
%


clc
clear
close all


v = VideoWriter('newfile');
open(v)
 
t0 = 0;
tf = 100;
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
   
    uav = uav.update_pos(vf);
    vf.avf{1}.x = vf.avf{1}.x+Vel*dt;
    vf.avf{1}.vx = Vel;
    vf = vf.xydomain(10,vf.avf{1}.x,vf.avf{1}.y,30);
    
    
    
    
    clf
    hold on
    
    
    vf.pltff
    vf.avf{1}.pltfnc
    uav.pltUAV
    
    axis equal
    axis([-30,30,-20,20]);
    grid on
       drawnow
    
           writeVideo(v,getframe(gcf));
    
end


close(v);





