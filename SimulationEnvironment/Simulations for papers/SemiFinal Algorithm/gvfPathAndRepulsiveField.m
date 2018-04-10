%=========================================================================
% gvfPathAndRepulsiveField.m
%
% Plots for goal path, repulsive field, and decay functions
%
%==========================================================================


clc
clear
close all



%Setup vector field
vf = vectorField();
vf = vf.xydomain(50,0,0,20);
%Goal Path
vf = vf.navf('line');
vf.avf{1}.angle = pi/2;
vf.avf{1}.H = 5;
vf.avf{1}.normComponents = false;
hold on
vf.pltff()
plot([-50,60],[0,0],'g','linewidth',3);
set(gca,'fontsize',12)
legend({'Guidance vector','Path'});
xticks(-50:10:50);
yticks(-50:10:50);
xlabel('x');
ylabel('y');
grid on
axis([-50,50,-50,50]);




%Setup vector field
vf = vectorField();
vf = vf.xydomain(50,0,0,20);

%Obstacle
vf = vf.nrvf('circ');
vf.rvf{1}.r = 25;
vf.rvf{1}.H = 0;
vf.rvf{1}.G = -1;
vf.rvf{1}.decayActive = false;

figure
hold on
vf.pltff()
vf.rvf{1}.pltfnc
set(gca,'fontsize',12)
legend({'Guidance vector','Path'});
xticks(-50:10:50);
yticks(-50:10:50);
xlabel('x');
ylabel('y');
grid on
axis([-50,50,-50,50]);



%Setup vector field
vf = vectorField();
vf = vf.xydomain(50,0,0,20);

%Obstacle
vf = vf.nrvf('circ');
vf.rvf{1}.r = 0.1;
vf.rvf{1}.H = 0;
vf.rvf{1}.G = -1;
vf.rvf{1}.decayActive = false;


figure
hold on
vf.pltff()
vf.rvf{1}.pltfnc
set(gca,'fontsize',12)
legend({'Guidance vector','Path'});
xticks(-50:10:50);
yticks(-50:10:50);
xlabel('x');
ylabel('y');
grid on
plot(0,0,'r.');
axis([-50,50,-50,50]);



%Setup vector field
vf = vectorField();
vf = vf.xydomain(50,0,0,20);

%Obstacle
vf = vf.nrvf('circ');
vf.rvf{1}.r = 0.1;
vf.rvf{1}.H = 1;
vf.rvf{1}.G = -1;
vf.rvf{1}.decayActive = true;
vf.rvf{1}.normComponents = true;
vf.rvf{1}.normTotal = true;
vf.NormSummedFields = false;
vf.rvf{1}.decayR = 35;


figure
hold on
vf.pltff()
vf.rvf{1}.pltfnc
set(gca,'fontsize',12)
plot(vf.rvf{1}.decayR*cos(0:0.1:2*pi),vf.rvf{1}.decayR*sin(0:0.1:2*pi),'k--','linewidth',2);

legend({'Guidance vector','Path','Decay Edge'});
xticks(-50:10:50);
yticks(-50:10:50);
xlabel('x');
ylabel('y');
grid on
axis([-50,50,-50,50]);


figure
range = 0:0.1:40;
p = -(tanh(2*pi*range/vf.rvf{1}.decayR-pi))+1;
plot(range,p,'linewidth',2);
grid on
set(gca,'fontsize',12);
xlabel('range');
ylabel('Activation Strength');





figure
%Setup vector field
vf = vectorField();

vf = vf.xydomain(50,0,0,20);

%Goal Path
vf = vf.navf('line');
vf.avf{1}.angle = pi/2;
vf.NormSummedFields = false;
vf.avf{1}.H = 5;
vf.avf{1}.normComponents = false;



%Obstacle
vf = vf.nrvf('circ');
vf.rvf{1}.decayR = 35;
vf.rvf{1}.r = 0.01;
vf.rvf{1}.H = 0;
vf.rvf{1}.G = -1;
vf.rvf{1}.y = 0;


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


vf = vf.xydomain(30,0,0,35);

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
















