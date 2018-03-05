%==========================================================================
% lookAheadSingularityDetection.m
%
%
%
%==========================================================================

clc
clear
close all


vf = vectorField();

%Goal Path
vf = vf.navf('line');
vf.avf{1}.angle = pi/2;
vf.NormSummedFields = 0;


%Obstacle
vf = vf.nrvf('circ');
vf.rvf{1}.r = 0.01;
vf.rvf{1} = vf.rvf{1}.modDecay('hyper');

%Setup singularity solver
f = @(X) VF(X,vf);
options = optimoptions('fsolve','Display','off');


ts = 0;
tf = 25;
dt = 0.1;
T = ts:dt:tf;
% T = linspace(ts,tf,60*tf);

xs = -10;
ys = 0;
v = 1;
heading = 0;

uav = UAV();
uav = uav.setup(xs,ys,v,heading,dt);

figure
hold


singx = [];
singy = [];
pause()
for i=1:length(T)
    cla;
    [u,v]=vf.heading(uav.x,uav.y);
    heading_cmd = atan2(v,u);
    uav = uav.update_pos(heading_cmd);
    
    Xics = uav.flightEnvX;
    Yics = uav.flightEnvY;
    
    tic
    for j=1:length(Xics)
        X0 = [Xics(j),Yics(j)];
        [location,gradMag,solverFlag] = fsolve(f,X0,options);
        
        if solverFlag > 0
           singx = [singx,location(1)];
           singy = [singy,location(2)];
        end  
    end
    1/toc
    
    plot(singx,singy,'ro','markersize',10,'markerfacecolor','k');
    vf.pltff;
    vf.rvf{1}.pltEqualStrength();
    fig = uav.pltUAV;
    drawnow()
    


    
end









function F = VF(X,vf)
    x = X(1);
    y = X(2);
    [F(1),F(2)] = vf.heading(x,y);
end

