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

plt=false;
f = @(X) VF(X,plt);

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

decayRadii    = 20/0.35:1:5*20/0.35;
circulations = 0.5:0.1:3;



tic
costs = [];
for j = 1:length(decayRadii)
    parfor k = 1:length(circulations)
        cost = f([decayRadii(j),circulations(k)]);
        
        rs(j,k) = decayRadii(j);
        hs(j,k) = circulations(k);
        costs(j,k) = cost;
    end
    
    str = strcat(num2str(j/length(decayRadii)*100), ' % complete at  ',num2str(floor(toc/60)),' minutes');
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




plt = true;
f = @(X) VF(X,plt);

X = [r,h];
f(X);


 
xlabel('x');
ylabel('y');
title('Lowest cost obstacle avoidance from search space');


function E = VF(X,plt)

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
    tf = 50;
    dt = 0.01;
    T = ts:dt:tf;
    
    xs = -200;
    ys = 0;
    v = 20;
    heading = 0;

    uav = UAV();
    uav = uav.setup(xs,ys,v,heading,dt);
    
    R = uav.v/0.35;
    vf.avf{1}.H = 2;
    e = 0;
    
    while uav.x<200
        [u,v]=vf.heading(uav.x,uav.y);
        heading_cmd = atan2(v,u);
        uav = uav.update_pos(heading_cmd);
        
        range = sqrt(uav.x^2+uav.y^2);
        
        if range < uav.v/0.35
            e = e+10/range  +  uav.y/(R)*dt;
        else
            e = e+uav.y/(R)*dt;
        end

    end
    
    if plt
    figure
    hold on
    uav.pltUAV();
    vf.pltff();
    vf.pltDecay();
    
    cxs = R*cos(0:0.1:2*pi);
     cys = R*sin(0:0.1:2*pi);
     plot(cxs,cys,'k.');
    
    vf.rvf{1}.pltEqualStrength();
    end
    E = e;

end








