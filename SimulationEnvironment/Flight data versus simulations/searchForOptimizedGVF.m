%=========================================================================
% searchForOptimizedGVF.m
%
% 
% Script for optimized GVF decay radius multiplier k and circulathion Ho.
% Results from thesis work 
%
%
%
%                               Method and code developed by: Garrett Clem
%==========================================================================


clc
clear
close all



%UAV initial position 
xs = -100;
xf = 125;
ys = 0;
velocity = 10;
heading = 0;
dt = 0.01;

%Obstacle Definition
n = 1;
obstR = n*velocity/0.35;
obstY = 0;

%Parameters to search

k   = linspace(1.5,5,300);
H_o = linspace(0.01,3,300);


plotFinal = false;
fr = @(X) GVF(X,velocity,dt,plotFinal,obstR,obstY,xs,ys,n,xf);

COSTS = [];
KS = [];
HS = [];
SIMTIME = [];

tic
for i=1:length(k)
    parfor j=1:length(H_o)
        X = [k(i),H_o(j)];
        [cost,isTrapped] = fr(X);
        
        
        COSTS(i,j) = cost;
        KS(i,j) = k(i);
        HS(i,j) = H_o(j);
       
        if isTrapped
            COSTS(i,j) = NaN;
            TRAPS(i,j) = cost;
        else
            TRAPS(i,j) = NaN;
        end
    end
    
    clc
    str = strcat(num2str(i/length(k)*100),{' '}, '% complete');
    disp(str);
end

Time_To_Solve = toc;

figure
hold on
surf(KS,HS,COSTS);
xlabel('k');
ylabel('h');
zlabel('cost');
% 
% plot3(KS,HS,TRAPS,'ro','markerfacecolor','r');

save('LargeSearchSpaceData');












function [GVFcost,isTrapped] = GVF(X,velocity,dt,plotFinal,obstR,obstY,xs,ys,n,xf)

isTrapped = false;
obstX = 0;

%Parameters to solve for
k = X(1);
H = X(2);

%Setup vector field
vf = vectorField();

%Goal Path
vf = vf.navf('line');
vf.avf{1}.angle = pi/2;
vf.NormSummedFields = true;
vf.avf{1}.H = velocity*n*2;
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
uav = UAV();
uav = uav.setup(xs,ys,velocity,heading,dt);

%UAV plot settings
uav.plotHeading = false;
uav.plotCmdHeading = false;
uav.plotUAV = false;
uav.plotUAVPath = true;
uav.plotFlightEnv = false;


%Set decay field strength based on gamma ratio
vf.rvf{1}.decayR = k*obstR;
vf.rvf{1} = vf.rvf{1}.modDecay('hyper');

%Cost function initially 0
cost = 0;

COST = [];
ERROR = [];

optPath = genOptPath(uav,obstR,0,obstY);
while uav.x<=xf
    
    range = sqrt((uav.x-vf.rvf{1}.x)^2+(uav.y-vf.rvf{1}.y)^2);
    
    [u,v]=vf.heading(uav.x,uav.y);
    heading_cmd = atan2(v,u);
    uav = uav.update_pos(heading_cmd);
    
    
    if uav.heading > deg2rad(175) && uav.heading <deg2rad(285)
        GVFcost = sum(COST);
        isTrapped = true;
        disp('trapped');
        return
    end
    
    [cost,error,location] = costANDerror(uav,obstR,obstX,obstY,optPath,dt);
    COST  = [COST;cost];
    ERROR = [ERROR;error];
    
    
end
GVFcost = sum(COST);
if plotFinal == true
    
    vf.NormSummedFields = true;
    vf =vf.xydomain(200,0,0,45);
    hold on
    
    cxs = obstR*cos(0:0.01:2.1*pi)+obstX;
        
    cys = obstR*sin(0:0.01:2.1*pi)+obstY;
    plotHeat = false;
    numericallyLocate = true;
    sing = locateSingularities(vf,plotHeat,numericallyLocate,obstR);
    
    
    [X,Y,U,V] = vf.sumFields();
    
    set(gca,'fontsize',12);
    p1 = quiver(X,Y,U,V);
    p2 = plot(cxs,cys,'linewidth',2);
    p6 = plot(sing(:,1),sing(:,2),'ro','markersize',10,'markerfacecolor','r');
    p7 = plot([uav.xs(1),175],[uav.ys(1),0],'g','linewidth',4);
    p5 = plot(uav.xs,uav.ys,'k-','linewidth',2);
    p3 = plot(uav.xs(1),uav.ys(1),'db','markersize',10,'markerfacecolor','b');
    p4 = plot(uav.xs(end),uav.ys(end),'sr','markersize',10,'markerfacecolor','r');

    p8 = vf.rvf{1}.pltEqualStrength;
    optPath = genOptPath(uav,obstR+1,vf.rvf{1}.x,vf.rvf{1}.y);
    % p8 = plot(optPath(:,1),optPath(:,2),'b-.','linewidth',3);
    
    legend([p1,p2,p3,p4,p5,p6,p7,p8],{'Guidance','Obstacle','UAV Start','UAV end','UAV Path','Singularity','Planned Path','Equal Strength'},'Location','southeast');
    axis equal
    axis([-125,200,-75,75]);
    
    str = strcat('m=',num2str(n),{'  '}, 'G=',num2str(-1),{'  '},'H=',num2str(sprintf('%0.1f',H)),{'  '},'k=',num2str(sprintf('%0.1f',k)),{'  '},'Cost=',num2str(sprintf('%0.0f',GVFcost)));
    title(str);
    xlabel('East [m]');
    ylabel('North [m]');
    
end




end












