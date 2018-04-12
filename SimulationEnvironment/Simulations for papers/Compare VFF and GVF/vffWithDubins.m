%==========================================================================
% vffWithDubins.m
%
%
%
%==========================================================================

clc
clear
close all

dt = 0.1;



goalPos = [50,0];
obstPos = [0,0];

uav = UAV();
uav = uav.setup(-20,0,1,0,dt);


VFF = vff();
VFF = VFF.setup(goalPos,obstPos);






uav.plotHeading = false;
uav.plotCmdHeading = false;
uav.plotUAV = false;
uav.plotUAVPath = true;
uav.plotFlightEnv = false;
uav.colorMarker = 'k--';




keepOutR = 5;
keepOutXs = keepOutR*cos(0:0.1:2*pi);
keepOutYs = keepOutR*sin(0:0.1:2*pi);



while uav.x<50
    


    VFF = VFF.heading(uav.x,uav.y);
    uav = uav.update_pos(VFF.cmd_heading);

   
end

hold on
plot([-20,50],[0,0],'g','linewidth',2);
uav.pltUAV();
plot(uav.xs(1),uav.ys(1),'d','markersize',10,'markerfacecolor','b');
plot(goalPos(1),goalPos(2),'go','markersize',10,'markerFaceColor','g');
plot(obstPos(1),obstPos(2),'ro','markerfacecolor','r');
plot(keepOutXs,keepOutYs,'r--','linewidth',2);

plot(VFF.detectionRadius*cos(0:0.1:2*pi),VFF.detectionRadius*sin(0:0.1:2*pi),'r-.','linewidth',0.25);
grid on
set(gca,'fontsize',16);
xlabel('x');
ylabel('y');
axis equal
legend({'Goal Path','UAV Path','UAV Start','Goal','Obstacle Center','Keep Out','Obstacle Window'});
axis([-25,55,-25,25]);










