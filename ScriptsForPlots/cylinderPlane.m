

clc
clear
close all
%Cylinder

figure
hold on
cylinder


p.x = -1.5:0.5:1.5;
p.y = p.x;

[x,y] = meshgrid(p.x,p.y);
z = 0.5*ones(length(x));

s = surf(x,y,z);
set(gca,'fontsize',16)
xlabel('x');
ylabel('y');
zlabel('z');
grid on
view([45,45]);

