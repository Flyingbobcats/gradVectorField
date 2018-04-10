%==========================================================================
% demonstrateDubinsObstacle.m
%
%
%
%==========================================================================

clc
clear
close all

velocity = 10;

figure
%Setup vector field
vf = vectorField();

vf = vf.xydomain(100,0,0,35);

%Goal Path
vf = vf.navf('line');
vf.avf{1}.angle = pi/2;
vf.NormSummedFields = false;
vf.avf{1}.H = velocity/0.35;
vf.avf{1}.normComponents = false;
vf.normAttractiveFields = false;



%Obstacle
vf = vf.nrvf('circ');
vf.rvf{1}.decayR = 18/0.35*1.8949;
vf.rvf{1}.r = 0.01;
vf.rvf{1}.H = 0.01;
vf.rvf{1}.G = -1;
vf.rvf{1}.y = 0;


obstR = velocity/0.35;
obstx = obstR*cos(0:0.1:2.1*pi)+vf.rvf{1}.x;
obsty = obstR*sin(0:0.1:2.1*pi)+vf.rvf{1}.y;



hold on
vf.pltff()
vf.rvf{1}.pltEqualStrength
plot(vf.rvf{1}.decayR*cos(0:0.1:2*pi),vf.rvf{1}.decayR*sin(0:0.1:2*pi),'k--','linewidth',2);
plot([-50,60],[0,0],'g','linewidth',3);

xticks(-50:10:50);
yticks(-50:10:50);
axis([-50,50,-50,50]);
set(gca,'fontsize',12);
xlabel('x');
ylabel('y');

legend({'Guidance vector','Equal Strength','Repulsive Edge','Path'});



figure
for i=1:length(vf.xspace)
    for j = 1:length(vf.yspace)

        [Ut,Vt] = vf.heading(vf.xspace(i),vf.yspace(j));
        mag = sqrt(Ut^2+Vt^2);
        
        Xs(i,j) = vf.xspace(i);
        Ys(i,j) = vf.yspace(j);
        Mag(i,j) = mag;
        
    end
end

surf(Xs,Ys,Mag);
set(gca,'fontsize',12);
xlabel('x');
ylabel('y');
zlabel('Guidance Vector Magnitude');
xticks(-50:25:50);
yticks(-50:25:50);
zticks(0:0.5:3);


%Create UAV class instance
uav = UAV();
uav = uav.setup(-100,0,10,0,0.1);

%UAV plot settings
uav.plotHeading = false;
uav.plotCmdHeading = false;
uav.plotUAV = false;
uav.plotUAVPath = true;
uav.plotFlightEnv = false;

figure
% hold on
while uav.x < 75 
    
    abs(uav.y)
    vf.avf{1}.H = velocity/(0.35*abs(uav.y));
    [u,v]=vf.heading(uav.x,uav.y);
    heading_cmd = atan2(v,u);
    uav = uav.update_pos(heading_cmd);
    clf
    hold on
    uav.pltUAV()
    vf.pltff();
    drawnow()
    

end

vf.pltff()
vf.rvf{1}.pltEqualStrength
plot(vf.rvf{1}.decayR*cos(0:0.1:2*pi),vf.rvf{1}.decayR*sin(0:0.1:2*pi),'k--','linewidth',2);
plot([-50,60],[0,0],'g','linewidth',3);
plot(obstx,obsty,'b');
xticks(-75:15:75);
yticks(-75:15:75);
axis([-75,75,-50,50]);
set(gca,'fontsize',12);
xlabel('x');
ylabel('y');














