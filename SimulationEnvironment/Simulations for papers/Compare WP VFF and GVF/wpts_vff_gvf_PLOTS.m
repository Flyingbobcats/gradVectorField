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
v = 25;
obstR = v/0.35+100;
obstY = 0;

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
options.DiffMaxChange = 1;
% options.PlotFcn = @optimplotfval;
options.StepTolerance = 1e-5;


x0 = [lbGamma,2];
A = [];
b = [];
Aeq = [];
beq = [];
lb = [lbGamma,1];
ub = [lbGamma*2,6];

saveFigure = true;
tic
[Xsolved,costR] = fmincon(fr,x0,A,b,Aeq,beq,lb,ub,[],options);
sim_time = toc;
plotFinal = true;




figure('pos',[10 10 900 600]);
fr = @(X) GVF(X,v,dt,plotFinal,obstR,obstY);

pltObst(0,obstY,obstR,x0,0,v,dt);
cost = fr(Xsolved);
set(gca,'fontsize',12);
xlabel('East (m)');
ylabel('North (m)');


x = -(v/0.35*Xsolved(1)+obstR)*1.1;
y = 0;

WPguidance(25,x,y,obstR,0,obstY,dt,Xsolved(1));
VFFguidance(v,x,y,obstR,0,obstY,dt,Xsolved(1));

axis([-550,550,-300,300]);
legend({'Obstacle','UAV Start','UAV End','Planned Path','GVF','Waypoint','VFF'});

function pltObst(obstX,obstY,obstR,x0,y0,velocity,dt)
uav = UAV();
uav = uav.setup(x0, y0,velocity, 0, dt);

obstx = obstR*cos(0:0.1:2.1*pi)+obstX;
obsty = obstR*sin(0:0.1:2.1*pi)+obstY;

hold on
optPath = genOptPath(uav,obstR,obstX,obstY);

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
xs =  -(velocity/0.35*gamma+obstR)*1.1;
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
vf.rvf{1}.decayR = uav.turn_radius*gamma+obstR;
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

while uav.x<=(uav.turn_radius*gamma+obstR)*1.1
    
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
        
        
        %If inside avoidance region, weight functions
        if abs(uav.x)<= abs(X*1.02) && alpha > atan2(Y_turn,X_turn)
            %Weight attractive and repulsive fields
            vf.rvfWeight = 1*activationFunctions(alpha,'r');
            vf.avfWeight = 1/8*activationFunctions(alpha,'a');
            
            vf.rvfWeight = 1;
            vf.avfWeight = 1/8;
            
            
            
            %Weight convergence term of repulsive field
            g = -velocity*cos(abs(beta))-abs(1/((range-obstR)*velocity));
            if g>0
                g = 0;
            end
            vf.rvf{1}.G = g;
            
            
            %Switch to attractive field when exiting avoidance region
            
            if uav.y<=Y_turn && uav.x>=X_turn
                vf.avfWeight = 1;
                vf.rvfWeight = 0;
            end
        else
            vf.avfWeight = 1;
            vf.rvfWeight = 0;
        end
        
        
        
        ALPHA = [ALPHA,alpha];
        BETA = [BETA,beta];
        RVFW = [RVFW,vf.rvfWeight];
        AVFW = [AVFW,vf.avfWeight];
        GS = [GS,vf.rvf{1}.G];
        
        
        %Calculate guidance from field and send to UAV
        [u,v]=vf.heading(uav.x,uav.y);
        heading_cmd = atan2(v,u);
        uav = uav.update_pos(heading_cmd);
    end
    
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
    uav.Marker = '-';
    uav.MarkerColor = 'r';
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


function WPguidance(velocity,x0,y0,obstR,obstX,obstY,dt,gamma)

uav = UAV();
uav.plotHeading = false;
uav.plotCmdHeading = false;

uav.plotUAV = false;
uav.plotUAVPath = true;
uav.plotFlightEnv = false;
uav = uav.setup(x0, y0,velocity, 0, dt);



optPath = genOptPath(uav,obstR,obstX,obstY);



wpMan = wpt();
wpMan.WPradius = 20;
optWaypoints = waypointPlanner(uav,obstR+wpMan.WPradius,obstX,obstY,20);
wpMan = wpMan.setup(optWaypoints);
wpMan.WPx(end+1)=(uav.turn_radius*gamma+obstR)*1.1;
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
uav.pltUAV();
axis equal
grid on

str = strcat('WPT: cost = ',num2str(sum(COST)),{'  '},'RMS Error = ', num2str(rms(ERROR)));
disp(str);
end

function VFFguidance(velocity,x0,y0,obstR,obstX,obstY,dt,gamma)


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
VFF = VFF.setup([(uav.turn_radius*gamma+obstR)*1.1,1],[0,obstY]);
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
while uav.x<=(uav.turn_radius*gamma+obstR)*1.1
    alpha = atan2(uav.y-obstY,uav.x);
    VFF = VFF.heading(uav.x,uav.y);
    
    uav = uav.update_pos(VFF.cmd_heading);
    

    
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


end



















