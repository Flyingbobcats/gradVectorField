
clc
clear
close all

vf = vectorField;
vf = vf.navf('line');
vf.avf{1}.angle = pi/2;

vf = vf.nrvf('circ');
vf.rvf{1}.r = 0.01;
vf.rvf{1} = vf.rvf{1}.modDecay('hyper');

vf.rvfWeight = 1;
vf = vf.xydomain(10,0,0,25);

vf.rvf{1}.H = 1;

vf.NormSummedFields = 0;
vf.normAttractiveFields = 0;
vf.rvf{1}.normComponents = 0;

[XT,YT,UT,VT] = vf.sumFields;
quiver(XT,YT,UT,VT);
axis([-10,10,-10,10]);
axis square
xlabel('x');
ylabel('y');

mag = [];
for i=1:length(UT)
    for j=1:length(VT)
        
        mag(i,j) = sqrt(UT(i,j)^2+VT(i,j)^2);
        
    end
end

figure
surf(XT,YT,mag)
grid on
xlabel('x');
ylabel('y');
zlabel('vector magnitude');



figure
hold on
vf.pltff
contour(XT,YT,mag,'showtext','on')
grid on
xlabel('x');
ylabel('y');
axis equal

