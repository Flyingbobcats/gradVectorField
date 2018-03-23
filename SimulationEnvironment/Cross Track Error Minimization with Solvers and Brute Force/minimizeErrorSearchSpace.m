%=========================================================================
% minimizeErrorSearchSpace.m
%
%  Evaluate possible decay radii and circulation weights for cherry-picked
%  scenario. Used to compare to minimizeErrorDecayFunc and
%  minimizeErrorCirculation programs
%
%==========================================================================



clc
clear
close all

f = @(X) VF(X);

% % options = optimoptions('fmincon','Display','final-detailed','Algorithm','interior-point','DiffMinChange',0.05,'PlotFcn',@optimplotfval);
% options = optimoptions('fmincon','Display','final-detailed','Algorithm','interior-point');
% options.DiffMinChange = 0.01;
% % options.DiffMaxChange = 1;
% options.PlotFcn = @optimplotresnorm;
% options.PlotFcn = @optimplotstepsize;
% options.PlotFcn = @optimplotfval;
% options.StepTolerance = 1e-5;
% 
% 
% x0 = [0.1];
% A = [];
% b = [];
% Aeq = [];
% beq = [];
% lb = [0.1];
% ub = [10];
% 
% h = fmincon(f,x0,A,b,Aeq,beq,lb,ub,[],options);
% disp(h)

decayRadii    = 5:0.001:10;
circulations = 0.1:0.01:2;

tic
costs = [];
for j = 1:length(decayRadii)
    parfor k = 1:length(circulations)
        cost = f([decayRadii(j),circulations(k)]);
        
        rs(j,k) = decayRadii(j);
        hs(j,k) = circulations(k);
        costs(j,k) = cost;
    end
    
    str = strcat(num2str(j/length(decayRadii)*100), ' % complete at_  ',num2str(toc/60),' minutes');
    disp(str);
end

minMatrix = min(costs(:));
[row,col] = find(costs==minMatrix);

r = decayRadii(row);
h = circulations(col);

figure
surf(rs,hs,costs);
xlabel('r');
ylabel('h');
zlabel('cost');





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
vf.rvf{1}.decayR = r(1);
vf.rvf{1} = vf.rvf{1}.modDecay('hyper');
vf.rvf{1}.H = h(1);

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
     vf.rvf{1}.pltEqualStrength();
 
 cxs = 2.5*cos(0:0.1:2*pi);
 cys = 2.5*sin(0:0.1:2*pi);
 plot(cxs,cys,'k.');
xlabel('x');
ylabel('y');
title('Lowest cost obstacle avoidance from search space');


function E = VF(X)

    r = X(1);
    H = X(2);
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
    vf.rvf{1}.decayR = r;
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








