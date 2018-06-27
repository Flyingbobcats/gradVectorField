%==========================================================================
% crossTrackErrorSearchSpace.m
%
% Evaluate a pre-defined space of circulation and obstacle radii and record
% the cross track error
%
%==========================================================================

clc
clear
close all




hs = 0:0.01:5;
rs = 2.5:0.01:10;

for j = 1:length(hs)
    parfor k = 1:length(rs)
        cost = VF(hs(j),rs(k));
        H(j,k) = hs(j);
        R(j,k) = rs(k);
        COST(j,k) = cost;
    end
    clc
    str = strcat(num2str(j/length(hs)*100), ' % complete');
    disp(str);
end


figure
surf(H,R,COST)
xlabel('H');
ylabel('decay radius');

        
 



function E = VF(H,R)

    h = H;
    r = R;
    
    vf = vectorField();

    %Goal Path
    vf = vf.navf('line');
    vf.avf{1}.angle = pi/2;
    vf.NormSummedFields = 0;


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
            e(i) = dt;
           
        else
            e(i) = 0;
        end

    end
%     if r == 2.5 && H==1
%         figure
%         hold on
%         vf.pltff
%         vf.rvf{1}.pltDecay
%         uav.pltUAV
%         pause()
%         
%     elseif r==10 && H == 1
%         figure
%         hold on
%         vf.pltff
%         vf.rvf{1}.pltDecay
%         uav.pltUAV
%         pause()
%         
%     end
    E = sum(e);
end








