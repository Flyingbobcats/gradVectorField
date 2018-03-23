%=========================================================================
% minimizeErrorDecayCirculation.m
%
% Minimize the cost function for occupying the avoidance region by
% modifying the circulation weight
%
%==========================================================================



clc
clear
close all

f = @(H) VF(H);

% options = optimoptions('fmincon','Display','final-detailed','Algorithm','interior-point','DiffMinChange',0.05,'PlotFcn',@optimplotfval);
options = optimoptions('fmincon','Display','final-detailed','Algorithm','interior-point');
options.DiffMinChange = 0.01;
% options.DiffMaxChange = 1;
options.PlotFcn = @optimplotresnorm;
options.PlotFcn = @optimplotstepsize;
options.PlotFcn = @optimplotfval;
options.StepTolerance = 1e-5;


x0 = [0.1];
A = [];
b = [];
Aeq = [];
beq = [];
lb = [0.1];
ub = [10];

[h,fval] = fmincon(f,x0,A,b,Aeq,beq,lb,ub,[],options);
disp(h)


vf = vectorField();
%Goal Path
vf = vf.navf('line');
vf.avf{1}.angle = pi/2;
vf.NormSummedFields = 0;
vf.avf{1}.normComponents = true;
vf.normAttractiveFields = false;

%Obstacle
vf = vf.nrvf('circ');
vf.rvf{1}.r = 0.01;
vf.rvf{1}.decayR =  6.4285;
vf.rvf{1} = vf.rvf{1}.modDecay('hyper');
vf.rvf{1}.H = h;

ts = 0;
tf = 25;
dt = 0.1;
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
end

figure
hold on
uav.pltUAV();
vf.pltff();
vf.pltDecay();

cxs = 2.5*cos(0:0.1:2*pi);
cys = 2.5*sin(0:0.1:2*pi);
plot(cxs,cys,'k.');
vf.rvf{1}.pltEqualStrength();


function E = VF(H)

    
    vf = vectorField();

    %Goal Path
    vf = vf.navf('line');
    vf.avf{1}.angle = pi/2;
    vf.NormSummedFields = 0;
    vf.avf{1}.normComponents = true;
    vf.normAttractiveFields = false;

    %Obstacle
    vf = vf.nrvf('circ');
    vf.rvf{1}.r = 0.01;
%     vf.rvf{1}.decayR = 6.4285;

    vf.rvf{1} = vf.rvf{1}.modDecay('hyper');
    vf.rvf{1}.H = H;

    ts = 0;
    tf = 25;
    dt = 0.1;
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
            e(i) = 100+uav.y;
        else
            e(i) = uav.y;
        end

    end
    
%     figure
%     hold on
%     uav.pltUAV();
%     vf.pltff();
%     vf.pltDecay();
%     
%     cxs = 2.5*cos(0:0.1:2*pi);
%      cys = 2.5*sin(0:0.1:2*pi);
%      plot(cxs,cys,'k.');
%     
%     vf.rvf{1}.pltEqualStrength();
%     pause()
    E = sum(e);

end








