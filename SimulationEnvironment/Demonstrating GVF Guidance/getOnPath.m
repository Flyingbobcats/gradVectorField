% getOnPath.m
%
% Demonstrate UAV getting onto straight GVF path
%



clc
clear
close all

obstacle = false;



xs = -150;
ys = 50;
heading = 45;
dt = 0.1;
velocity = 20;

vf = vectorField();

%Goal Path
vf = vf.navf('line');
vf.avf{1}.angle = pi/2;
vf.NormSummedFields = false;
vf.avf{1}.H = 20;
vf.avf{1}.normComponents = false;
vf.normAttractiveFields = false;
vf = vf.xydomain(150,0,0,30);



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

while uav.x<150
        [u,v]=vf.heading(uav.x,uav.y);
        heading_cmd = atan2(v,u);
        uav = uav.update_pos(heading_cmd); 
    
end

figure
hold on
set(gca,'fontsize',12);
vf.pltff();
plot([-50,50],[0,0],'g','linewidth',3);
uav.pltUAV();
plot(uav.xs(1),uav.ys(1),'db','markersize',10,'markerfacecolor','b');
plot(uav.xs(end),uav.ys(end),'sr','markersize',10,'markerfacecolor','r');
xlabel('x');
ylabel('y');
legend({'Guidance','Path','UAV Path','UAV Start','UAV End'});
axis([-200,200,-100,100]);
xticks(-50:10:50);








