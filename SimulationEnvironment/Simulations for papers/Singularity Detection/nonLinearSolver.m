%Testing non-linear systems solver for finding singularities
%


clc
clear
close all



runCirc = false;
runGridDensity = false;
runGridWidth = false;

% xs = -50:1:50;
% ys = xs;
% 
% for i=1:length(xs)
%     for j = 1:length(ys)
%         X = [xs(i),ys(j)];
%         F = VF(X);
%         US(i,j) = F(1);
%         VS(i,j) = F(2);
%         XS(i,j) = xs(i);
%         YS(i,j) = ys(j);
%         
%         mag(i,j) = sqrt(US(i,j)^2+VS(i,j)^2);
%         
%         US(i,j) = US(i,j)/mag(i,j);
%         VS(i,j) = VS(i,j)/mag(i,j);
%     end
% end
% 
% figure('pos',[10 10 800 800]);
% surf(XS,YS,mag)
% 
% xlabel('x');
% ylabel('y');
% 
% h = colorbar;
% ylabel(h, 'Vector Magnitude')
% shading interp
% view([0,90])
% set(gca,'fontsize',12);
% axis equal
% axis([-50,50,-50,50]);

xs = -50:3:50;
ys = xs;
hold on
for i=1:length(xs)
    for j = 1:length(ys)
        X = [xs(i),ys(j)];
        F = VF(X);
        US(i,j) = F(1);
        VS(i,j) = F(2);
        XS(i,j) = xs(i);
        YS(i,j) = ys(j);
        ZS(i,j) = 3;
        WS(i,j) = 0;
        
        mag(i,j) = sqrt(US(i,j)^2+VS(i,j)^2);
        
        US(i,j) = US(i,j)/mag(i,j);
        VS(i,j) = VS(i,j)/mag(i,j);
    end
end

quiver3(XS,YS,ZS,US,VS,WS,'k');


if runCirc
%Circular field
options = optimoptions('fsolve','Display','off','Algorithm','levenberg-marquardt');%,'UseParallel',true);
R = 10:1:30;


figure

for j = 1:length(R)
    figure
    ops.m = 10;
    ops.n = 10;
    ops.xlimit = 10;
    ops.ylimit = 10;
    ops.r = R(j);
    ops.d_theta = deg2rad(10);
    XYS = icPoints('circle',ops);

    fun = @VF;
    location = cell(1,length(XYS));
    gradMag  = cell(1,length(XYS));
    solverFlag = cell(1,length(XYS));
    
    
%     subplot(length(R)/5,5,j)


    hold on
    p9 = quiver(XS,YS,US,VS);
    
    
    
    
    tic
    for i =1:length(XYS)
        X0 = [XYS(1,i),XYS(2,i)];
        [location{i},gradMag{i},solverFlag{i}] = fsolve(fun,X0,options);
    end
    update_frequency = 1/toc;
%     str = strcat('Update Frequency = ',num2str(update_frequency),'Hz');
    theta = 0:0.1:2*pi;
    cxs = 35/2*cos(theta);
    cys = 35/2*sin(theta);
%     title(str)
    p7 = plot(cxs,cys,'r--','linewidth',2);
    p8 = plot([-50,60],[0,0],'g','linewidth',3);

    
 
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
        axis([-50,50,-50,50]);
        
        
    end
    legend([p9,p7,p4,p5,p8],{'Guidance Vector','Equal Strength','Initial Condition','Singularity','Path'});
    xticks(-50:10:50);
    yticks(-50:10:50);
    axis([-50,50,-50,50]);
    set(gca,'fontsize',12);
    xlabel('x');
    ylabel('y');
end
end


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
%             p1 = plot(x0,y0,'ro','markersize',7);
%             p2 = plot(x(1),x(2),'r.','markersize',30);
%             p3 = plot([x(1),x0],[x(2),y0],'r--','markersize',15);
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

[U,V] = vf.heading(x,y);

F(1) = U;
F(2) = V;

% mag = sqrt(F(1)^2+F(2)^2);
% F(1) = F(1)/mag;
% F(2) = F(2)/mag;
end















