%Testing non-linear systems solver

clc
clear
close all



runCirc = true;
runGridDensity = true;
runGridWidth = true;

xs = -10:1:10;
ys = xs;

for i=1:length(xs)
    for j = 1:length(ys)
        X = [xs(i),ys(j)];
        F = VF(X);
        US(i,j) = F(1);
        VS(i,j) = F(2);
        XS(i,j) = xs(i);
        YS(i,j) = ys(j);
        
        mag(i,j) = sqrt(US(i,j)^2+VS(i,j)^2);
        
        US(i,j) = US(i,j)/mag(i,j);
        VS(i,j) = VS(i,j)/mag(i,j);
    end
end

figure
surf(XS,YS,mag)



if runCirc
%Circular field
options = optimoptions('fsolve','Display','off','Algorithm','levenberg-marquardt');%,'UseParallel',true);
R = .5:.5:5;


figure

for j = 1:length(R)
    ops.m = 10;
    ops.n = 10;
    ops.xlimit = 10;
    ops.ylimit = 10;
    ops.r = R(j);
    ops.d_theta = deg2rad(5);
    XYS = icPoints('circle',ops);

    fun = @VF;
    location = cell(1,length(XYS));
    gradMag  = cell(1,length(XYS));
    solverFlag = cell(1,length(XYS));
    
    
    subplot(length(R)/5,5,j)


    hold on
    quiver(XS,YS,US,VS);
    
    
    
    
    tic
    for i =1:length(XYS)
        X0 = [XYS(1,i),XYS(2,i)];
        [location{i},gradMag{i},solverFlag{i}] = fsolve(fun,X0,options);
    end
    update_frequency = 1/toc;
    str = strcat('Update Frequency = ',num2str(update_frequency),'Hz');
    theta = 0:0.1:2*pi;
    cxs = 2.5*cos(theta);
    cys = 2.5*sin(theta);
    title(str)
    plot(cxs,cys,'r--','linewidth',2)

    
    
    for i =1:length(XYS)
        x0 = XYS(1,i);
        y0 = XYS(2,i);
        x = location{i};
        if solverFlag{i} == -2
            p1 = plot(x0,y0,'ro','markersize',7);
            p2 = plot(x(1),x(2),'r.','markersize',30);
            p3 = plot([x(1),x0],[x(2),y0],'r--','markersize',15);
        elseif solverFlag{i} ==1 || solverFlag{i} ==2 || solverFlag{i} ==3 || solverFlag{i} ==4
            p4 = plot(x0,y0,'ko','markersize',7);
            p5 =plot(x(1),x(2),'k.','markersize',20);
            p6 =plot([x(1),x0],[x(2),y0],'k--','markersize',15);
        end
        axis equal
        axis([-6,6,-6,6]);
        
        
    end
end
end
tightfig

if runGridDensity
%Grid fixed size with different density
options = optimoptions('fsolve','Display','off');
ns = 3:1:10;
for j = 1:length(ns)
    ops.m = ns(j);
    ops.n = ns(j);
    ops.xlimit = 10;
    ops.ylimit = 10;
    XYS = icPoints('grid',ops);
    grid on
    axis equal
    
    
    
    fun = @VF;
    
    location = cell(1,length(XYS));
    gradMag  = cell(1,length(XYS));
    solverFlag = cell(1,length(XYS));
    
    
    tic
    parfor i =1:length(XYS)
        X0 = [XYS(1,i),XYS(2,i)];
        if X0(1) ~= 0 || X0(2) ~=0
            [location{i},gradMag{i},solverFlag{i}] = fsolve(fun,X0,options);
        end
    end
    1/toc
    
    
    figure
    hold on
    quiver(XS,YS,US,VS);
    axis equal
    
    for i =1:length(XYS)
        x0 = XYS(1,i);
        y0 = XYS(2,i);
        x = location{i};
        
        if isempty(x0)|| isempty(y0)|| isempty(x)
%             disp('igorning IC');
        else
        if solverFlag{i} == -2
            p1 = plot(x0,y0,'ro','markersize',7);
            p2 = plot(x(1),x(2),'r.','markersize',30);
            p3 = plot([x(1),x0],[x(2),y0],'r--','markersize',15);
        elseif solverFlag{i} ==1
            p4 = plot(x0,y0,'ko','markersize',7);
            p5 =plot(x(1),x(2),'k.','markersize',45);
            p6 =plot([x(1),x0],[x(2),y0],'k--','markersize',15);
        end
        end
        
    end
end
end

if runGridWidth
%Grid with fixed density and varying size
%Grid fixed size with different density
options = optimoptions('fsolve','Display','off');
ws = 1:1:10;
for j = 1:length(ns)
    ops.m = 5;
    ops.n = 5;
    ops.xlimit = ws(j);
    ops.ylimit = ws(j);
    XYS = icPoints('grid',ops);
    grid on
    axis equal
   
    fun = @VF;
    
    location = cell(1,length(XYS));
    gradMag  = cell(1,length(XYS));
    solverFlag = cell(1,length(XYS));
    
    
    tic
    parfor i =1:length(XYS)
        X0 = [XYS(1,i),XYS(2,i)];
        if X0(1) ~= 0 || X0(2) ~=0
            [location{i},gradMag{i},solverFlag{i}] = fsolve(fun,X0,options);
        end
    end
    1/toc
    
    
    figure
    hold on
    quiver(XS,YS,US,VS);
    axis equal
    
    for i =1:length(XYS)
        x0 = XYS(1,i);
        y0 = XYS(2,i);
        x = location{i};
        
        if isempty(x0)|| isempty(y0)|| isempty(x)
%             disp('igorning IC');
        else
        if solverFlag{i} == -2
            p1 = plot(x0,y0,'ro','markersize',7);
            p2 = plot(x(1),x(2),'r.','markersize',30);
            p3 = plot([x(1),x0],[x(2),y0],'r--','markersize',15);
        elseif solverFlag{i} ==1
            p4 = plot(x0,y0,'ko','markersize',7);
            p5 =plot(x(1),x(2),'k.','markersize',45);
            p6 =plot([x(1),x0],[x(2),y0],'k--','markersize',15);
        end
        end
        
    end
end
end






function F = VF(X)
%Compute values of each vector component
x = X(1);
y = X(2);
%Constants
theta = deg2rad(90);
a = cos(theta);
b = sin(theta);

xc = 0;
yc = 0;
r = 0.1;

decayR = 5;


UG = -(a*x+b*y)*a+b;
VG = -(a*x+b*y)*b-a;
magG = sqrt(UG^2+VG^2);


H = 0;
G = 1;

UO = G*2*(x-xc)*((x-xc)^2+(y-yc)^2-r^2)+H*2*(y-yc);
VO = G*2*(y-yc)*((x-xc)^2+(y-yc)^2-r^2)-H*2*(x-xc);
magO = sqrt(UO^2+VO^2);


ug = UG/magG;
vg = VG/magG;

uo = UO/magO;
vo = VO/magO;

r = sqrt((x-xc)^2+(y-yc)^2);
p = -(tanh(2*pi*r/decayR-pi))+1;

F(1) = ug+p*uo;
F(2) = vg+p*vo;

% mag = sqrt(F(1)^2+F(2)^2);
% F(1) = F(1)/mag;
% F(2) = F(2)/mag;
end
