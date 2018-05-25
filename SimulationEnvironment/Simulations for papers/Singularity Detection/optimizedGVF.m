%=========================================================================
% findSingularities.m
%
% Script for finding singularities in the vector field and plotting a UAVs
% path in that vector field.
%
%
%==========================================================================


clc
clear
close all

% Setup Vector Field



xstart = -150;
ys = 0;
velocity = 10;
heading = 0;
dt = 0.1;

uav = UAV();
uav = uav.setup(xstart,0,velocity,heading,dt);
uav.plotHeading = false;
uav.plotCmdHeading = false;
uav.plotUAV = false;
uav.plotUAVPath = true;
uav.plotFlightEnv = false;
% uav.colorMarker = 'k-';

%Solved
% n = 1.5;
% k = 3.201;
% H = 2.9971;

%Solved
n = 1.5;
k = 2.79;
H = 1.7;

n = 1.5;
k = 2.8;
H = 2.03;


%Brute Force without circulation
% n = 1.5;
% k = 2.8;
% H = 0;


%Brute Force with circulation
% n = 1.5;
% k = 2.8;
% H = 1;

obstR = n*uav.turn_radius;
obstY = -1/2*obstR;


decayR = k*n*uav.turn_radius;




vf = vectorField();

vf = vf.xydomain(200,0,0,20);

%Goal Path
vf = vf.navf('line');
vf.avf{1}.angle = pi/2;
vf.NormSummedFields = false;
vf.avf{1}.H = velocity;
vf.avf{1}.normComponents = false;



%Obstacle
vf = vf.nrvf('circ');
vf.rvf{1}.decayR = decayR;
vf.rvf{1}.r = 0.01;
vf.rvf{1}.H = H;
vf.rvf{1}.G = -1;
vf.rvf{1}.y = -1;




xs = -100:10:100;
ys = xs;

for i=1:length(xs)
    for j = 1:length(ys)
        X = [xs(i),ys(j)];
        F = VF(X,vf);
        US(i,j) = F(1);
        VS(i,j) = F(2);
        XS(i,j) = xs(i);
        YS(i,j) = ys(j);
        
        mag(i,j) = sqrt(US(i,j)^2+VS(i,j)^2);
        
        US(i,j) = US(i,j)/mag(i,j);
        VS(i,j) = VS(i,j)/mag(i,j);
    end
end

figure('pos',[10 10 900 500]);
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


hold on
for i=1:length(xs)
    for j = 1:length(ys)
        X = [xs(i),ys(j)];
        F = VF(X,vf);
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





%Circular field
options = optimoptions('fsolve','Display','off','Algorithm','levenberg-marquardt');%,'UseParallel',true);
R = decayR/2;





ops.m = 10;
ops.n = 10;
ops.xlimit = 10;
ops.ylimit = 10;
ops.r = R;
ops.d_theta = deg2rad(10);
XYS = icPoints('circle',ops);

fun = @(X) VF(X,vf);
location = cell(1,length(XYS));
gradMag  = cell(1,length(XYS));
solverFlag = cell(1,length(XYS));






hold on
vf.NormSummedFields = true;
[X,Y,U,V] = vf.sumFields();
vf.NormSummedFields = false;


theta = 0:0.1:2.1*pi;
cxs = obstR*cos(theta);
cys = obstR*sin(theta);

p1 = quiver(X,Y,U,V,'linewidth',1.25);
p7 = plot([xstart-50,-xstart+50],[0,0],'color',[0.5 0.5 0.5],'linewidth',3);

p2 = plot(cxs,cys,'r-','linewidth',3);


for i =1:length(XYS)
    X0 = [XYS(1,i),XYS(2,i)];
    [location{i},gradMag{i},solverFlag{i}] = fsolve(fun,X0,options);
end



for i =1:length(XYS)
    x0 = XYS(1,i);
    y0 = XYS(2,i);
    x = location{i};
    if solverFlag{i} == -2
        %plot nothing, this was from older versions
    elseif solverFlag{i} ==1 || solverFlag{i} ==2 || solverFlag{i} ==3 || solverFlag{i} ==4
        p6 =plot(x(1),x(2),'ro','markersize',10,'markerfacecolor','r');
    end

    
    
end

set(gca,'fontsize',12);
xlabel('East [m]');
ylabel('North [m]');

cost = 0;
while uav.x<175
    [u,v]=vf.heading(uav.x,uav.y);
    heading_cmd = atan2(v,u);
    uav = uav.update_pos(heading_cmd);
    
    cost = cost+ abs(uav.y) / ((obstR))*dt;
    
%     uav.pltUAV();
%     drawnow();
    
end



p3 = plot(uav.xs(1),uav.ys(1),'db','markersize',10,'markerfacecolor','b');
p4 = plot(uav.xs(end),uav.ys(end),'sr','markersize',10,'markerfacecolor','r');
p5 = plot(uav.xs,uav.ys,'k-','linewidth',2);
p8 = vf.rvf{1}.pltEqualStrength;
optPath = genOptPath(uav,obstR+1,vf.rvf{1}.x,vf.rvf{1}.y);
% p8 = plot(optPath(:,1),optPath(:,2),'b-.','linewidth',3);

legend([p1,p2,p3,p4,p5,p6,p7,p8],{'Guidance','Obstacle','UAV Start','UAV end','UAV Path','Singularity','Planned Path','Equal Strength'},'Location','southeast');
axis equal
axis([-200,200,-100,100]);

str = strcat('n=',num2str(n),{'  '}, 'G=',num2str(-1),{'  '},'H=',num2str(H),{'  '},'R=',num2str(floor(decayR)),'m',{'  '},'Cost=',num2str(floor(14)));
title(str);




function F = VF(X,vf)
x = X(1);
y = X(2);
[U,V] = vf.heading(x,y);
F(1) = U;
F(2) = V;
end




function GVFcost = GVF(X,velocity,dt,obstR,obstY,xs,ys,plotFinal)




%Parameters to solve for
k = X(1);
H = X(2);

%Setup vector field
vf = vectorField();

%Goal Path
vf = vf.navf('line');
vf.avf{1}.angle = pi/2;
vf.NormSummedFields = false;
vf.avf{1}.H = velocity;
vf.avf{1}.normComponents = false;
vf.normAttractiveFields = false;


%Obstacle
vf = vf.nrvf('circ');
vf.rvf{1}.r = 0.01;
vf.rvf{1}.H = H;
vf.rvf{1}.G = -1;
vf.rvf{1}.y = obstY;

%Setup UAV initial position (x will be changed later)
heading = 0;


%Create UAV class instance
uav = UAV();
uav = uav.setup(xs,ys,velocity,heading,dt);

%UAV plot settings
uav.plotHeading = false;
uav.plotCmdHeading = false;
uav.plotUAV = false;
uav.plotUAVPath = true;
uav.plotFlightEnv = false;

vf.rvf{1}.decayR = k*obstR;
vf.rvf{1} = vf.rvf{1}.modDecay('hyper');

%Cost function initially 0
cost = 0;
COST = [];
ERROR = [];
optPath = genOptPath(uav,obstR,0,obstY);
while uav.x<=400
    range = sqrt((uav.x-vf.rvf{1}.x)^2+(uav.y-vf.rvf{1}.y)^2);
    
    %Calculate guidance from field and send to UAV
    [u,v]=vf.heading(uav.x,uav.y);
    heading_cmd = atan2(v,u);
    uav = uav.update_pos(heading_cmd);
    
    [cost,error,location] = costANDerror(uav,obstR,obstX,obstY,optPath);
    COST  = [COST;cost];
    ERROR = [ERROR;error];
    
    if uav.heading > deg2rad(175) && uav.heading <deg2rad(285)
        GVFcost = sum(COST);
        print('trapped');
        return
    end
end


if plotFinal
    p3 = plot(uav.xs(1),uav.ys(1),'db','markersize',10,'markerfacecolor','b');
    p4 = plot(uav.xs(end),uav.ys(end),'sr','markersize',10,'markerfacecolor','r');
    p5 = plot(uav.xs,uav.ys,'k-','linewidth',2);
    p8 = vf.rvf{1}.pltEqualStrength;
    optPath = genOptPath(uav,obstR+1,vf.rvf{1}.x,vf.rvf{1}.y);
    % p8 = plot(optPath(:,1),optPath(:,2),'b-.','linewidth',3);
    
    legend([p1,p2,p3,p4,p5,p6,p7,p8],{'Guidance','Obstacle','UAV Start','UAV end','UAV Path','Singularity','Planned Path','Equal Strength'},'Location','southeast');
    axis equal
    axis([-200,200,-100,100]);
    
    str = strcat('n=',num2str(n),{'  '}, 'G=',num2str(-1),{'  '},'H=',num2str(H),{'  '},'R=',num2str(floor(decayR)),'m',{'  '},'Cost=',num2str(floor(14)));
    title(str);
end


GVFcost = sum(COST);
end














