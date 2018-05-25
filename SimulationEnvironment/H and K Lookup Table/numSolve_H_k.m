%=========================================================================
% numSolve_H_K.m
%
%
%==========================================================================



clc
clear
close all


NS = [];
VS = [];
COSTS = [];
XSOLVED = {};


ns = 1:0.5:5;
vs = 1:20:100;

ns = [1,2,3,4];
vs = [10,20,30,40];

ns = 1:0.5:5;
vs = 10:2:100;

ns = 1.5;
vs = 10;

% ks = 1:0.1:5;
% hs = 1:0.1:10;

% ks = 3;
% hs = 2.9;

for k=1:length(vs)
    for j =1:length(ns)
        
        
        figure
        n = ns(j);
        v = vs(k);
    
        obstR = n*v/0.35;
        obstY = 0;
        tr = v/0.35;
        dt = 0.01;
        
        plotFinal = false;
        
        fr = @(X) VF(X,v,dt,plotFinal,obstR,obstY,n);         %Find min with respect to r
        
        
        
        options = optimoptions('fmincon','Display','off');
        options.DiffMinChange = 0.01;
        options.DiffMaxChange = 0.2;
%         options.PlotFcn = @optimplotfval;
        options.StepTolerance = 1e-5;
        
        x0 = [2.75,3];
        A = [];
        b = [];
        Aeq = [];
        beq = [];
        lb = [2.5,3.2];
        ub = [3,3.5];
        
        
        
        tic
%         [Xsolved,costR] = fmincon(fr,x0,A,b,Aeq,beq,lb,ub,[],options);
        sim_time = toc;
        
%         Xsolved = [hs(j),ks(k)];
          Xsolved = [4,4];   %No circ example in thesis
        
        
        plotFinal = true;
        fr = @(X) VF(X,v,dt,plotFinal,obstR,obstY,n);
        cost = fr(Xsolved);
        % disp(Xsolved)
        % disp(cost)
%         str = strcat('h = ',num2str(Xsolved(1)),{' '}, '/gamma =',num2str(2*n),{' '},'cost=',num2str(cost),{' '},'time=',num2str(sim_time),'k=',num2str(Xsolved(2)));
        
        str = strcat('H = ',num2str(Xsolved(1)),{' '}, 'k = ', num2str(Xsolved(2)),{' '},'cost=',num2str(cost),{' '},'n = ', num2str(n));
%         title(str)
        
        
        
        NS(k,j) = ns(j);
        VS(k,j) = vs(k);
%         if cost>10
%             cost = 10;
%         end
        KS(k,j) = Xsolved(2);
        HS(k,j) = Xsolved(1);
        COSTS(k,j) = cost;
        XSOLVED{k,j} = Xsolved;
%         
%         
%     end
%     clc

% end

    end
        str = strcat('Percent complete:', num2str((100*k)/(length(vs))));
        disp(str);
end

% figure
% surf(VS,NS,HS);
% xlabel('v');
% ylabel('n');
% zlabel('hs');
% 
% figure
% surf(VS,NS,KS);
% xlabel('v');
% ylabel('n');
% zlabel('ks');
% 
% 
% figure;
% surf(VS,NS,COSTS);
% xlabel('v');
% ylabel('n');
% zlabel('costs');







function cost = VF(X,velocity,dt,plotFinal,obstR,obstY,n)

k = X(2);
H = X(1);
gamma = 1.75*n;

%Setup vector field
vf = vectorField();

vf = vf.xydomain(200,0,0,25);

%Goal Path
vf = vf.navf('line');
vf.avf{1}.angle = pi/2;
vf.NormSummedFields = false;
%         vf.avf{1}.H = 2*H*velocity^1/2;%*velocity;
vf.avf{1}.H = obstR;
vf.avf{1}.normComponents = false;
vf.normAttractiveFields = false;


%Obstacle
vf = vf.nrvf('circ');
vf.rvf{1}.r = 0.01;
vf.rvf{1}.H = H;
vf.rvf{1}.G = -1;
vf.rvf{1}.y = obstY;

%Obstacle (no fly zone radius)
%         obstR = velocity/0.35+150;
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
uav.colorMarker = 'k-';




%Change UAVs starting position to just outside the decay field edge


%Set decay field strength based on gamma ratio, plus obstacles
%radius

%         vf.rvf{1}.decayR = uav.turn_radius*gamma+obstR;
%         vf.rvf{1}.decayR = uav.turn_radius*(3*n);
vf.rvf{1}.decayR = uav.turn_radius*(k*n);

vf.rvf{1} = vf.rvf{1}.modDecay('hyper');

%Cost function initially 0
cost = 0;

cost = 0;
while uav.x<=175
    
    [u,v]=vf.heading(uav.x,uav.y);
    heading_cmd = atan2(v,u);
    
    if uav.heading > deg2rad(175) && uav.heading <deg2rad(285)
        cost = 1000;
        break
    end
    uav = uav.update_pos(heading_cmd);
    
    
    %Deviation from path cost, normalized with respect to obstacles
    %radius
    cost = cost+ abs(uav.y) / ((vf.rvf{1}.decayR))*dt;
    
    range = sqrt((uav.x-vf.rvf{1}.x)^2+(uav.y-vf.rvf{1}.y)^2);
    if range < obstR
        cost = cost+100;
    end
    
    
end
if plotFinal == true
    clf
    hold on
    
    vf.NormSummedFields = true;
    vf.pltff();
        p3 = plot([-250,250],[0,0],'g','linewidth',2);

    
    p1 = plot(obstx,obsty,'r','linewidth',2);
    p2 = plot(uav.xs(1),uav.ys(1),'db','markersize',10,'markerfacecolor','b');
    p2 = plot(uav.xs(end),uav.ys(end),'s','markersize',10,'markerfacecolor','r');

    p4 = uav.pltUAV();
    vf.rvf{1}.pltEqualStrength();
    
    
    xlabel('x');
    ylabel('y');
    set(gca,'fontsize',12);
    
    axis equal
    grid on
    axis([-200,200,-100,100]);
    
    
    
    
    
    legend({'Guidance','Planned Path','Obstacle','UAV Start','UAV End', 'UAV Path','Equal Strength'});
end


end











