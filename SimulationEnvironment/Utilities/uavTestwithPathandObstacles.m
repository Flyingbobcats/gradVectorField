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
tf = 250;
dt = .1;

t = t0:dt:tf;

vf = vectorField;
vf = vf.navf('circ');
vf.avf{1}.r = (1/0.35)*1.5;

vf = vf.nrvf('circ');
vf.rvf{1} = vf.rvf{1}.modDecay('hyper');
vf.rvf{1}.decayR = 5;
vf.rvf{1}.y = 10;
vf.rvf{1}.x = 5;
vf.rvfWeight = 50;
vf.rvf{1}.r = 0.01;
% vf.rvf{1}.H = -0.5;

vf = vf.nrvf('circ');
vf.rvf{2} = vf.rvf{1}.modDecay('hyper');
vf.rvf{2}.decayR = 5;
vf.rvf{2}.y = 5;
vf.rvf{2}.x = 15;
vf.rvfWeight = 50;
vf.rvf{2}.r = 0.01;




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
    
    vec.x(i) =vf.avf{1}.x;
    vec.y(i) =vf.avf{1}.y;
 
    
    vf = vf.xydomain(10,vf.avf{1}.x,vf.avf{1}.y,30);
    

%     clf
%     hold on
%     
% 
%     vf.avf{1}.pltfnc
%     uav.pltUAV
%     vf.rvf{1}.pltDecay
%  
% 
%     vf.rvf{2}.pltDecay
%     
%     plot(vec.x,vec.y,'--k','linewidth',2)
%     axis equal
%     axis([-30,30,-20,20]);
%     grid on
%     drawnow
    
%     writeVideo(v,getframe(gcf));
    error(i) = sqrt((uav.x-vec.x(i))^2+(uav.y-vec.y(i))^2)-vf.avf{1}.r;
end

hold on
plot(vec.x,vec.y,'--k','linewidth',2);
plot(uav.xs,uav.ys,'b.');
vf.rvf{1}.pltff
vf.rvf{2}.pltff
axis([-10,40,-20,20]);
axis equal
% close(v);

figure
plot(uav.ts(2:end),error)





