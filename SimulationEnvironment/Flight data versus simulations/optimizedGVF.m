%=========================================================================
% optimizedGVF.m
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

numSolved = true;
compareToCF = true;

if compareToCF
    xs = -2;
    xf = 2;
    ys = 0;
    velocity = 0.2;
    dt = 0.1;
    heading = 0;
    n=1.5;
    obstR = n*velocity/0.35;
    obstY =0.5*obstR;


else
%UAV initial position 
ys = 0;
velocity = 15;
heading = 0;
dt = 0.1;

%Obstacle Definition
n = 5;
obstR = n*velocity/0.35;
obstY =1/2*obstR;

xs = -100*n;
xf = 125*n;


%Brute Force without circulationf
% n = 1.5;
% k = 2.8;
% H = 0;

%Brute Force with circulation
% n = 1.5;
% k = 2.8;
% H = 1;
end





% 
if numSolved
plotFinal = false;
fr = @(X) GVF(X,velocity,dt,plotFinal,obstR,obstY,xs,ys,n,xf);

% Optimization options
options = optimoptions('fmincon','Display','final-detailed');
options = optimoptions('fmincon','Display','final-detailed');
options.DiffMinChange = 0.1;
options.DiffMaxChange = 0.2;
options.PlotFcn = @optimplotfval;
options.StepTolerance = 1e-3;


x0 = [2,2];
A = [];
b = [];
Aeq = [];
beq = [];
lb = [2,1];
ub = [4,6];

saveFigure = true;
tic
[Xsolved,costR] = fmincon(fr,x0,A,b,Aeq,beq,lb,ub,[],options);
sim_time = toc;

else


Xsolved = [2.95,0.1];
Xsolved = [2.95,1];
end
plotFinal = true;



disp('gvf parameters');
Xsolved
figure('pos',[10 10 900 600]);
fr = @(X) GVF(X,velocity,dt,plotFinal,obstR,obstY,xs,ys,n,xf);
fr(Xsolved);





function GVFcost = GVF(X,velocity,dt,plotFinal,obstR,obstY,xs,ys,n,xf)


obstX = 0;

%Parameters to solve for

k = X(1);

if obstY>0
    H = -X(2);
else
    H = X(2);
end


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
    vf =vf.xydomain(xs*(n+1),0,0,45);
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
    
    try
        p6 = plot(sing(:,1),sing(:,2),'ro','markersize',10,'markerfacecolor','r');
    catch
        warning('error plotting singularities');
    end
    p7 = plot([uav.xs(1),175*n],[uav.ys(1),0],'g','linewidth',4);
    p5 = plot(uav.xs,uav.ys,'k-','linewidth',2);
    p3 = plot(uav.xs(1),uav.ys(1),'db','markersize',10,'markerfacecolor','b');
    p4 = plot(uav.xs(end),uav.ys(end),'sr','markersize',10,'markerfacecolor','r');

    p8 = vf.rvf{1}.pltEqualStrength;
    optPath = genOptPath(uav,obstR+1,vf.rvf{1}.x,vf.rvf{1}.y);
    % p8 = plot(optPath(:,1),optPath(:,2),'b-.','linewidth',3);
    
    try
        h = legend([p1,p2,p3,p4,p5,p6,p7,p8],{'Guidance','Obstacle','UAV Start','UAV end','UAV Path','Singularity','Planned Path','Equal Strength'},'Location','best');
        h.Position=[0.745555557095342 0.250416670441627 0.159999996920427 0.15];
    catch
        warning('Error in plotting, may be due to no singularities');
    end
    axis equal
    axis([xs*(n+1),xf*(n+1),-(obstR)*(n+1),(obstR)*(n+1)]);
    
    str = strcat('m=',num2str(n),{'  '}, 'G=',num2str(-1),{'  '},'H=',num2str(sprintf('%0.1f',H)),{'  '},'k=',num2str(sprintf('%0.1f',k)),{'  '},'Cost=',num2str(sprintf('%0.0f',GVFcost)));
    title(str);
    xlabel('East [m]');
    ylabel('North [m]');
    
end




end
















