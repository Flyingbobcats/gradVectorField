
function [position,velocity,t,wpts] = post()

filename = uigetfile('.txt');
% disp(filename)
DATA = importFile(filename,1,100000000);

% retrieving UAV position from DATA and storing it in 'position'
position(:,1) = DATA(:,2); % x coordinates
position(:,2) = DATA(:,3); % y coordinates
position(:,3) = DATA(:,4); % z coordinates
% yaw_rate = DATA(:,24);
yaw_rate = DATA(:,5);

velocity = DATA(:,6);

% retrieving time from DATA
t = DATA(:,1);

% retrieving waypoint position from DATA and storing it in 'waypoint'
waypoint(:,1) = DATA(:,7);
waypoint(:,2) = DATA(:,8);
waypoint(:,3) = DATA(:,9);

% storing unique waypoints in 'wpts'
wpts = unique(waypoint,'rows');

% plot of UAV path with waypoints
figure
hold on
plot3(position(:,1),position(:,2),position(:,3))            % plotting the UAV path
% plot3(wpts(:,1),wpts(:,2),wpts(:,3),'*r')    % plotting the waypoints
xlabel('x');
ylabel('y');
zlabel('z');
axis square
grid on
view([45,45]);
axis equal

% x position
figure('pos',[1,1,700,1000]);
subplot(4,1,1)
hold on
grid on
set(gca,'fontsize',12);
plot(t,position(:,1)) % plotting x position versus time
xlabel('time time [s]');
ylabel('x [m]');
plot(t,waypoint(:,1),'--r','linewidth',2); % plotting waypoint x path
legend({'Measured state','Set-point'});

% y position
subplot(4,1,2)
hold on
grid on
set(gca,'fontsize',12);
plot(t,position(:,2)) % plotting y position versus time
xlabel('time [s]');
ylabel('y [m]');
plot(t,waypoint(:,2),'--r','linewidth',2); % plotting waypoint y path

% z position
subplot(4,1,3)
set(gca,'fontsize',12);
hold on
grid on
plot(t,position(:,3)) % plotting y position versus time
xlabel('time [s]');
ylabel('z [m]');
plot(t,waypoint(:,3),'--r','linewidth',2); % plotting waypoint y path

subplot(4,1,4)
hold on
set(gca,'fontsize',12);
grid on
plot(t,rad2deg(yaw_rate)) % plotting y position versus time
xlabel('time [s]');
ylabel('yaw rate [deg/s]');
plot(t,zeros(1,length(t)),'--r','linewidth',2); % plotting waypoint y path


figure
plot(t,velocity)
xlabel('time (seconds)');
ylabel('speed (m/s');





m = 1.5;
theta_r = 0.2/0.35;

CXS = m*theta_r*cos(0:0.01:2*pi);
CYS = m*theta_r*sin(0:0.01:2*pi)+m*0.5*theta_r;
figure
hold on
plot(position(:,1),position(:,2))            % plotting the UAV path
% plot(wpts(:,1),wpts(:,2),'*r')    % plotting the waypoints
plot(CXS,CYS,'k--');
xlabel('x');
ylabel('y');
zlabel('z');
axis square
grid on
axis equal

end






