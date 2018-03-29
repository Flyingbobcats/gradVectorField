%==========================================================================
% flightEnvelope.m
%
% From inputs (x,y,heading,heading_rate, speed, and time)
% Determine UAV flight envelope
% - Possible locations of UAV at time t horizon
%==========================================================================

clc
clear
close all

dt = 0.001;
ts = 0;
tf = 1;

T = ts:dt:tf;

uav = UAV();

v = 11;             %25 mph

heading = 0;

x = 0;
y = 0;

% uav.setup(x,y,v,heading,dt)  

headings = -pi/2:0.05:pi/2;
XYS = [];
figure
hold on
for i=1:length(headings)
    heading = headings(i);
    uav = UAV();
    uav = uav.setup(x,y,v,0,dt);
    uav.plotHeading = 0;
    uav.plotCmdHeading = 0;
    
    
    for j = 1:length(T)
        uav = uav.update_pos(heading);
%         uav.pltUAV()
%         rad2deg(uav.heading())
%         pause()
        
%         pause()
    
    end
    uav.pltUAV();
    plot(uav.xs(end),uav.ys(end),'ro');
    
    XYS = [XYS; uav.xs(end),uav.ys(end)];
    
end

axis equal
xlabel('x (meters)');
ylabel('y (meters)');
title('Fixed wing UAV flight envolope t_h=5 seconds')
grid on


figure
plot(XYS(:,1),XYS(:,2),'k.');
axis equal
xlabel('x (meters)');
ylabel('y (meters)');
title('Fixed wing UAV flight envolope t_h=5 seconds')
grid on


