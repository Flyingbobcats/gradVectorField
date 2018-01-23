
% Single obstacle avoidance while tracking a straight line for various UAV
% velocities
%
%
%
%
%

clc
clear
close all

pltPaths = true;


V = [0.6,0.7,0.8,0.9,1];

costs = {};
for i = 1:length(V)
    vf = vectorField;
    vf = vf.navf('line');
    vf.avf{1}.angle = pi/2;
    vf.avf{1}.y = 0.01;
    
    vf = vf.xydomain(20,0,0,35);
    
    vf = vf.nrvf('circ');
    vf.rvf{1} = vf.rvf{1}.modDecay('hyper');
    vf.rvf{1}.r = 0.01;
    vf.rvf{1}.decayR = 5;
    vf.rvfWeight = 20;
    vf.rvf{1}.H = 0;
    
    uav = UAV;
    uav = uav.setup(-20,0,V(i),0,0.1);
    
    while uav.x<20
        uav = uav.update_pos(vf);
        if  uav.t > 500
            break
        end
    end
    xys{i,1} = uav.xs;
    xys{i,2} = uav.ys;
    
    cost = 0;
    for j=2:length(uav.xs)
        cost = cost+sqrt((uav.xs(j-1)-uav.xs(j))^2+(uav.ys(j-1)-(uav.ys(j))^2));
    end
    COST(i) = cost;
end


if pltPaths
    hold on
    vf.pltff
    vf.rvf{1}.pltDecay
    for i=1:length(xys)
        p(i) = plot(xys{i,1},xys{i,2},'linewidth',2);
    end
    
    lgd = legend([p(1),p(2),p(3),p(4),p(5)],{'0.6','0.7','0.8','0.9','1'},'location','southeast','FontSize',6);
    title(lgd,'UAV Velocities');
    axis([-20,20,-10,10]);
    set(gcf, 'PaperPosition', [0 0 5 2.5]); %Position plot at left hand corner with width 5 and height 5.
    set(gcf, 'PaperSize', [5 2.5]); %Set the paper to have width 5 and height 5.
    saveas(gcf, 'singleObstacle', 'pdf') %Save figure
end











