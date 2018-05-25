%==========================================================================
% wpts_vff_gvf.m
%
% compare the following methods for the general obstacle avoidance case:
%   Waypoints
%   Virtual Force Field
%   Gradient Vector Field
%
%
%
%==========================================================================

clc
clear
close all

clc
clear
close all


%Setup basic vehicle and obstacle parameters
v = 10;
obstR = v/0.35*2;
obstY = -1/2*obstR;

%Time step
dt = 0.1;

%Numerical solver bounds for gamma
tr = v/0.35;                                %Turn radius
lbGamma = (obstR / (tr));                   %Repulsive field no smaller than obstacle's radius

plotFinal = false;
fr = @(X) GVF(X,v,dt,plotFinal,obstR,obstY);

%Optimization options
% options = optimoptions('fmincon','Display','final-detailed');
options = optimoptions('fmincon','Display','final-detailed');
options.DiffMinChange = 0.1;
options.DiffMaxChange = 0.2;
options.PlotFcn = @optimplotfval;
options.StepTolerance = 1e-4;


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
sim_time = toc
plotFinal = true;


disp('gvf parameters');
Xsolved
figure('pos',[10 10 900 600]);
fr = @(X) GVF(X,v,dt,plotFinal,obstR,obstY);

OptPathCost = pltObst(0,obstY,obstR,x0,0,v,dt);
GVFcost = fr(Xsolved);
set(gca,'fontsize',12);
xlabel('East [m]');
ylabel('North [m]');


% X = [Xsolved(1),0];
% figure('pos',[10 10 900 600]);
% fr = @(X) GVFNoCirc(X,v,dt,plotFinal,obstR,obstY);
% 
% pltObst(0,obstY,obstR,x0,0,v,dt);
% GVFcost = fr(Xsolved);
% set(gca,'fontsize',12);
% xlabel('East (m)');
% ylabel('North (m)');








x = -400;
y = 0;

WPcost = WPguidance(25,x,y,obstR,0,obstY,dt,Xsolved(1));
VFFcost = VFFguidance(v,x,y,obstR,0,obstY,dt,Xsolved(1));

axis([-450,450,-175,300]);
legend({'Obstacle','UAV Start','UAV End','Planned Path','GVF','Waypoint','VFF'});


figure
hold on


c = {'VFF','GVF','WP','OPTIMAL'};
p1 = bar([1],[VFFcost]);
p2 = bar([2],[GVFcost]);
p3 = bar([3],[WPcost]);
p4 = bar([4],[OptPathCost]);

set(p1,'facecolor',[0.9100    0.4100    0.1700]);
set(p3,'facecolor','b');
set(p2,'facecolor','r');



set(gca,'fontsize',12);
xticks([1,2,3,4]);
xticklabels(c);
ylabel('Cost [-]');
axis([0,5,0,40]);





function optPathCost = pltObst(obstX,obstY,obstR,x0,y0,velocity,dt)
uav = UAV();
uav = uav.setup(x0, y0,velocity, 0, dt);

obstx = obstR*cos(0:0.1:2.1*pi)+obstX;
obsty = obstR*sin(0:0.1:2.1*pi)+obstY;

hold on
optPath = genOptPath(uav,obstR,obstX,obstY);

%Calculate cost of optimal path


dr = uav.v*dt;

i = 1;
cost = 0;
r = 0;

TotalRange = 0;
intermediateRange = 0;

while i<length(optPath)-1
    
    x1 = optPath(i,1);
    y1 = optPath(i,2);
    x2 = optPath(i+1,1);
    y2 = optPath(i+1,2);
    
    r = sqrt((x2-x1)^2+(y2-y1)^2);
    intermediateRange = intermediateRange + r;
    
    if intermediateRange>=dr
        cost = cost + abs(y2) / (obstR)*dt;
        intermediateRange = 0;
    end
    i=i+1;

    
end
% for i=1:length(optPath)
%     y = optPath(i,2);
%     x = optPath(i,1);
%     cost = cost + abs(y) / (obstR)*dt;
%     range = sqrt((x-obstX)^2+(y-obstY)^2);
%     %Pentalize for entering obstacle region
% %     if range < obstR
% %         cost = cost+100;
% %     end
% end


optPathCost = cost;



plot(obstx,obsty,'k','linewidth',1);


% plot(optPath(:,1),optPath(:,2),'r-','linewidth',5);

end
function GVFcost = GVF(X,velocity,dt,plotFinal,obstR,obstY)


plotFlight = false;

%Parameters to solve for
gamma = X(1);
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

%Obstacle (no fly zone radius)
obstx = obstR*cos(0:0.1:2.1*pi)+vf.rvf{1}.x;
obsty = obstR*sin(0:0.1:2.1*pi)+vf.rvf{1}.y;


%Setup UAV initial position (x will be changed later)
xs =  -400;
ys = 0;
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


%Set decay field strength based on gamma ratio
vf.rvf{1}.decayR = gamma*obstR;
vf.rvf{1} = vf.rvf{1}.modDecay('hyper');

%Cost function initially 0
cost = 0;


RANGE = [];
AVFW = [];
RVFW = [];
BETA = [];
ALPHA = [];
GS = [];


COST = [];
ERROR = [];

optPath = genOptPath(uav,obstR,0,obstY);

while uav.x<=400


    if uav.x<=(uav.turn_radius*gamma+obstR)*1.1
        %Weighting function independent variables
        alpha = atan2(uav.y-obstY,uav.x);
        beta = pi - alpha+uav.heading;
        range = sqrt((uav.x-vf.rvf{1}.x)^2+(uav.y-vf.rvf{1}.y)^2);
        
        %Calculate hard-turn point from optimal path
        turnR = uav.turn_radius;
        obstX = 0;
        y = turnR*(1-cos(pi/2));
        X = obstX - sqrt((turnR+obstR)^2-(y-obstY)^2);
        theta = asin((y-obstY)/(obstR+turnR));
        zeta = pi+theta;
        X_turn = (-X+turnR*cos(zeta));
        Y_turn = y+turnR*sin(zeta);
        
        
%         %If inside avoidance region, weight functions
%         if abs(uav.x)<= abs(X*1.02) && alpha > atan2(Y_turn,X_turn)
%             %Weight attractive and repulsive fields
%             vf.rvfWeight = 1*activationFunctions(alpha,'r');
%             vf.avfWeight = 1/8*activationFunctions(alpha,'a');
%             
%             vf.rvfWeight = 1;
%             vf.avfWeight = 1/8;
%             
%             
%             
%             %Weight convergence term of repulsive field
%             g = -velocity*cos(abs(beta))-abs(1/((range-obstR)*velocity));
%             if g>0
%                 g = 0;
%             end
%             vf.rvf{1}.G = g;
%             
%             
%             %Switch to attractive field when exiting avoidance region
%             
%             if uav.y<=Y_turn && uav.x>=X_turn
%                 vf.avfWeight = 1;
%                 vf.rvfWeight = 0;
%             end
%         else
%             vf.avfWeight = 1;
%             vf.rvfWeight = 0;
%         end
        
        
        
        ALPHA = [ALPHA,alpha];
        BETA = [BETA,beta];
        RVFW = [RVFW,vf.rvfWeight];
        AVFW = [AVFW,vf.avfWeight];
        GS = [GS,vf.rvf{1}.G];
end
        
        %Calculate guidance from field and send to UAV

        [u,v]=vf.heading(uav.x,uav.y);
        heading_cmd = atan2(v,u);
        uav = uav.update_pos(heading_cmd);
%     end
    
    if uav.heading > deg2rad(175) && uav.heading <deg2rad(285)
        GVFcost = sum(COST);
        print('trapped');
        return
    end
   
        [cost,error,location] = costANDerror(uav,obstR,obstX,obstY,optPath);
        COST  = [COST;cost];
        ERROR = [ERROR;error];

    
end

if plotFinal == true
%     uav.Marker = '-';
%     uav.MarkerColor = 'r';
    uav.linewidth = 2;

    
    plot(uav.xs(1),uav.ys(1),'db','markersize',10,'markerfacecolor','b');
    plot(uav.xs(end),uav.ys(end),'sr','markersize',10,'markerfacecolor','r');
    plot([uav.xs(1),uav.xs(end)],[0,0],'color',[0.4 0.4 0.4],'linewidth',3);
    uav.pltUAV();
    axis equal
    grid on
        
    
    str = strcat('GVF: cost = ',num2str(sum(COST)),{'  '},'RMS Error = ', num2str(rms(ERROR)));
    disp(str) 
end

GVFcost = sum(COST);


end

function F = magVF(X,vf)

[U,V]=vf.heading(X(1),X(2));
F(1) = U;
F(2) = V;

end


function c = WPguidance(velocity,x0,y0,obstR,obstX,obstY,dt,gamma)

uav = UAV();
uav.plotHeading = false;
uav.plotCmdHeading = false;

uav.plotUAV = false;
uav.plotUAVPath = true;
uav.plotFlightEnv = false;
uav = uav.setup(x0, y0,velocity, 0, dt);



optPath = genOptPath(uav,obstR,obstX,obstY);



wpMan = wpt();
wpMan.WPradius = 10;
optWaypoints = waypointPlanner(uav,obstR+wpMan.WPradius,obstX,obstY,30);

length(optWaypoints)

% optWaypoints = [-225+uav.turn_radius,0; 0,obstR+uav.turn_radius;200,0;500,0]
wpMan = wpMan.setup(optWaypoints);
wpMan.WPx(end+1)=400;
wpMan.WPy(end+1) = 0;

ERROR = [];
COST = [];

turnR = uav.turn_radius;
obstX = 0;
y = turnR*(1-cos(pi/2));
X = obstX - sqrt((turnR+obstR)^2-(y-obstY)^2);
theta = asin((y-obstY)/(obstR+turnR));
zeta = pi+theta;
X_turn = (-X+turnR*cos(zeta));
Y_turn = y+turnR*sin(zeta);



while wpMan.currentWP <= length(wpMan.WPx) && wpMan.active
    alpha = atan2(uav.y-obstY,uav.x);
    wpMan = wpMan.getWPT(uav.x,uav.y);
    heading = atan2(wpMan.wpy-uav.y,wpMan.wpx-uav.x);
    uav = uav.update_pos(heading);
    
    if abs(uav.x)<= abs(X*1.02) && alpha > atan2(Y_turn,X_turn)
        [cost,error,location] = costANDerror(uav,obstR,obstX,obstY,optPath);
        COST  = [COST ,cost];
        ERROR = [ERROR,error];
    end
    
end




hold on


uav.Marker = ':';
uav.MarkerColor = 'b';
uav.linewidth = 3;
% uav.pltUAV();
plot(uav.xs,uav.ys,'b:','linewidth',3);
axis equal
grid on

str = strcat('WPT: cost = ',num2str(sum(COST)),{'  '},'RMS Error = ', num2str(rms(ERROR)));
disp(str);

c = sum(COST);
end

function c = VFFguidance(velocity,x0,y0,obstR,obstX,obstY,dt,gamma)


uav = UAV();
uav.plotHeading = false;
uav.plotCmdHeading = false;

uav.plotUAV = false;
uav.plotUAVPath = true;
uav.plotFlightEnv = false;
uav = uav.setup(x0, y0,velocity, 0, dt);

obstx = obstR*cos(0:0.1:2.1*pi)+obstX;
obsty = obstR*sin(0:0.1:2.1*pi)+obstY;


optPath = genOptPath(uav,obstR,obstX,obstY);



VFF = vff();
VFF = VFF.setup([400,0],[0,obstY-0.1]);
VFF.detectionRadius = obstR+uav.turn_radius;


ERROR = [];
COST = [];

turnR = uav.turn_radius;
obstX = 0;
y = turnR*(1-cos(pi/2));
X = obstX - sqrt((turnR+obstR)^2-(y-obstY)^2);
theta = asin((y-obstY)/(obstR+turnR));
zeta = pi+theta;
X_turn = (-X+turnR*cos(zeta));
Y_turn = y+turnR*sin(zeta);

%    while abs(uav.x)<= abs(X*1.02) && alpha > atan2(Y_turn,X_turn)
while uav.x<=400
    alpha = atan2(uav.y-obstY,uav.x);
    VFF = VFF.heading(uav.x,uav.y);
    
    uav = uav.update_pos(VFF.cmd_heading);
    
    if uav.x>-20
        VFF.detectionRadius = 0;
    end
    
    if abs(uav.x)<= abs(X*1.02) && alpha > atan2(Y_turn,X_turn)
        [cost,error,location] = costANDerror(uav,obstR,obstX,obstY,optPath);
        COST  = [COST ,cost];
        ERROR = [ERROR,error];
    end
    
end

uav.Marker = '-.';
uav.MarkerColor = [0.9100    0.4100    0.1700];
uav.linewidth = 3;


uav.pltUAV();





axis equal
grid on
axis([-(uav.turn_radius*gamma+obstR)*1.1,(uav.turn_radius*gamma+obstR)*1.1,-(uav.turn_radius*gamma+obstR)*1.1,(uav.turn_radius*gamma+obstR)*1.1]);

str = strcat('VFF: cost = ',num2str(sum(COST)),{'  '},'RMS Error = ', num2str(rms(ERROR)));

disp(str)
c = sum(COST);
end


function GVFcost = GVFNoCirc(X,velocity,dt,plotFinal,obstR,obstY)



plotFlight = false;

%Parameters to solve for
gamma = X(1);
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
vf.rvf{1}.H = 0;
vf.rvf{1}.G = -1;
vf.rvf{1}.y = 0;

%Obstacle (no fly zone radius)
obstx = obstR*cos(0:0.1:2.1*pi)+vf.rvf{1}.x;
obsty = obstR*sin(0:0.1:2.1*pi)+vf.rvf{1}.y;


%Setup UAV initial position (x will be changed later)
xs =  -700;
ys = 0;
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


%Set decay field strength based on gamma ratio
vf.rvf{1}.decayR = gamma*obstR;
vf.rvf{1} = vf.rvf{1}.modDecay('hyper');

%Cost function initially 0
cost = 0;


RANGE = [];
AVFW = [];
RVFW = [];
BETA = [];
ALPHA = [];
GS = [];


COST = [];
ERROR = [];

optPath = genOptPath(uav,obstR,0,obstY);

while uav.x<=400


    if uav.x<=(uav.turn_radius*gamma+obstR)*1.1
        %Weighting function independent variables
        alpha = atan2(uav.y-obstY,uav.x);
        beta = pi - alpha+uav.heading;
        range = sqrt((uav.x-vf.rvf{1}.x)^2+(uav.y-vf.rvf{1}.y)^2);
        
        %Calculate hard-turn point from optimal path
        turnR = uav.turn_radius;
        obstX = 0;
        y = turnR*(1-cos(pi/2));
        X = obstX - sqrt((turnR+obstR)^2-(y-obstY)^2);
        theta = asin((y-obstY)/(obstR+turnR));
        zeta = pi+theta;
        X_turn = (-X+turnR*cos(zeta));
        Y_turn = y+turnR*sin(zeta);
          
        
        ALPHA = [ALPHA,alpha];
        BETA = [BETA,beta];
        RVFW = [RVFW,vf.rvfWeight];
        AVFW = [AVFW,vf.avfWeight];
        GS = [GS,vf.rvf{1}.G];
end
        
        %Calculate guidance from field and send to UAV

        [u,v]=vf.heading(uav.x,uav.y);
        heading_cmd = atan2(v,u);
        uav = uav.update_pos(heading_cmd);
%     end
    
    if uav.heading > deg2rad(175) && uav.heading <deg2rad(285)
        break
    end
    
    if abs(uav.x)<= abs(X*1.02) && alpha > atan2(Y_turn,X_turn)
        [cost,error,location] = costANDerror(uav,obstR,obstX,obstY,optPath);
        COST  = [COST;cost];
        ERROR = [ERROR;error];
    end
    
end

if plotFinal == true
    


%     uav.Marker = '-';
%     uav.MarkerColor = 'r';
    uav.linewidth = 2;
    vf = vf.xydomain(400,0,0,35);
    plot(uav.xs(1),uav.ys(1),'db','markersize',10,'markerfacecolor','b');
    plot(uav.xs(end),uav.ys(end),'sr','markersize',10,'markerfacecolor','r');
    plot([uav.xs(1),uav.xs(end)],[0,0],'color',[0.4 0.4 0.4],'linewidth',3);
    uav.pltUAV();
    axis equal
    grid on
        
    
    str = strcat('GVF: cost = ',num2str(sum(COST)),{'  '},'RMS Error = ', num2str(rms(ERROR)));
    disp(str) 
end

GVFcost = sum(COST);


end




















