


clc
clear
close all


xs = -1:0.5:1;
ys = xs;

[X,Y] = meshgrid(xs,ys);

Z = zeros(length(xs),length(xs));

[X1,Y1,Z1] = meshgrid(xs,zeros(length(xs),1),xs);

figure
hold on
g = surf(X,Y,Z,'Facecolor',[.148,.148,.148]);
g.FaceAlpha = 0.85;
xlabel('x');
ylabel('y');
zlabel('z');

h = surf(X,Z,Y,'Facecolor','b');
h.FaceAlpha = 0.85;
view([-45,45]);
axis equal


plot3([-1,1],[0,0],[0,0],'r','linewidth',5);
legend({'\alpha_1','\alpha_2','Path'});
set(gca,'fontsize',12);


