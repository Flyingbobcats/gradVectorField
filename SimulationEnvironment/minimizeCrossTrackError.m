%==========================================================================
% minimizeCrossTrackError.m
%
%
%
%==========================================================================

clc
clear
close all

f = @(H) VF(H);



% options = optimoptions('fmincon','Display','final-detailed','Algorithm','interior-point','DiffMinChange',0.05,'PlotFcn',@optimplotfval);
options = optimoptions('fmincon','Display','final-detailed','Algorithm','interior-point');
options.DiffMinChange = 0.1;
% options.DiffMaxChange = 1;
options.PlotFcn = @optimplotresnorm;
options.PlotFcn = @optimplotstepsize;
options.PlotFcn = @optimplotfval;
options.StepTolerance = 1e-5;







% H = [0.1,5];
% [x,fval] = fminsearch(f,H);

% [x,fval] = fsolve(f,H ,options);


x0 = [1,1];
A = [];
b = [];
Aeq = [];
beq = [];
lb = [0,2.5];
ub = [5,10];

[x,fval] = fmincon(f,x0,A,b,Aeq,beq,lb,ub,[],options);


h = x(1);
r = x(2);


figure
vf = vectorField();

%Goal Path
vf = vf.navf('line');
vf.avf{1}.angle = pi/2;
vf.NormSummedFields = 0;
vf.avf{1}.normComponents = false;
vf.normAttractiveFields = false;


%Obstacle
vf = vf.nrvf('circ');
vf.rvf{1}.r = 0.01;
vf.rvf{1} = vf.rvf{1}.modDecay('hyper');
vf.rvf{1}.H = h;
vf.rvf{1}.decayR = r;




ts = 0;
tf = 25;
dt = 0.1;
T = ts:dt:tf;

cxs = 2.5*cos(0:0.1:2*pi);
cys = 2.5*sin(0:0.1:2*pi);


xs = -10;
ys = 0;
v = 1;
heading = 0;

uav = UAV();
uav = uav.setup(xs,ys,v,heading,dt);
uav.plotHeading = false;
uav.plotCmdHeading = false;
uav.plotUAVPath = true;
uav.plotFlightEnv = false;
hold on
vf.rvf{1}.pltDecay();
vf.pltff;
plot(cxs,cys,'r--');

for i=1:length(T)
    [u,v]=vf.heading(uav.x,uav.y);
    heading_cmd = atan2(v,u);
    uav = uav.update_pos(heading_cmd);
    uav.pltUAV;
    drawnow;
end


        
 



function E = VF(H)

    h = H(1);
    r = H(2);
    
    vf = vectorField();

    %Goal Path
    vf = vf.navf('line');
    vf.avf{1}.angle = pi/2;
    vf.NormSummedFields = 0;
    vf.avf{1}.normComponents = false;
    vf.normAttractiveFields = false;

    %Obstacle
    vf = vf.nrvf('circ');
    vf.rvf{1}.r = 0.01;
    vf.rvf{1} = vf.rvf{1}.modDecay('hyper');
    vf.rvf{1}.H = h;
    vf.rvf{1}.decayR = r;
    

    

    ts = 0;
    tf = 25;
    dt = 0.01;
    T = ts:dt:tf;
    

    xs = -10;
    ys = 0;
    v = 1;
    heading = 0;

    uav = UAV();
    uav = uav.setup(xs,ys,v,heading,dt);
    

    for i=1:length(T)
        [u,v]=vf.heading(uav.x,uav.y);
        heading_cmd = atan2(v,u);
        uav = uav.update_pos(heading_cmd);
        
        range = sqrt(uav.x^2+uav.y^2);
        if range < 2.5
            e(i) = 10+uav.y;
       
        else
            e(i) = uav.y;
        end
%         e(i) = uav.y*dt;
    end
    
%     figure
%     hold on
%     uav.pltUAV();
%     vf.pltff();
%     vf.pltDecay();
%     
%     vf.rvf{1}.pltEqualStrength();
%     pause()
    E = sum(e);
end








