%=========================================================================
% decayRatioAndVelocity.m
%
% Is there a relationship between decay radius and uav velocity
%==========================================================================



clc
clear
close all

vs = 1:0.01:3;
rs = 2.5:0.01:20;

VS = [];
RS = [];
costs = [];

for j = 1:length(vs)
    tic
    parfor k =1:length(rs)
        H = 1;                        %Fix H, assume circulation direction
        ts = 0;
        tf = 35;
        dt = 0.1;
        T = ts:dt:tf;


        xs = -30;
        ys = 0;
        v = vs(j);
        r = rs(k);
        heading = 0;
        uav = UAV();
        uav = uav.setup(xs,ys,v,heading,dt);

        fr = @(r,H,v,dt) VF(r,H,v,dt);         %Find min with respect to r

        
        VS(j,k) = v;
        RS(j,k) = r;
        costs(j,k) = fr(r,H,v,dt);

    end
    clc
    time_remaining = toc*(length(vs)-j)/60;
    str = strcat(num2str(j/length(vs)*100), '% complete. Approx. Time Remaining: ',{' '},num2str(time_remaining),' minutes');
    disp(str);
end

figure
surf(VS/0.35,RS,costs);
xlabel('Turning Radius');
ylabel('decay radius');
zlabel('cost');


% vf = vectorField();
% %Goal Path
% vf = vf.navf('line');
% vf.avf{1}.angle = pi/2;
% vf.NormSummedFields = 0;
% vf.avf{1}.normComponents = false;
% vf.normAttractiveFields = false;
% 
% %Obstacle
% vf = vf.nrvf('circ');
% vf.rvf{1}.r = 0.01;
% vf.rvf{1}.decayR = decayR;
% vf.rvf{1} = vf.rvf{1}.modDecay('hyper');
% vf.rvf{1}.H = H;
% 
% 
% 
% 
% for i=1:length(T)
%     [u,v]=vf.heading(uav.x,uav.y);
%     heading_cmd = atan2(v,u);
%     uav = uav.update_pos(heading_cmd);
% end
% 
% figure
% hold on
% uav.pltUAV();
% vf.pltff();
% vf.pltDecay();
% 
% cxs = 2.5*cos(0:0.1:2*pi);
% cys = 2.5*sin(0:0.1:2*pi);
% plot(cxs,cys,'k.');
% vf.rvf{1}.pltEqualStrength();


function E = VF(r,H,v,dt)

    
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
    vf.rvf{1}.decayR = r;
    vf.rvf{1} = vf.rvf{1}.modDecay('hyper');
    vf.rvf{1}.H = H;

    ts = 0;
    tf = 25;
    T = ts:dt:tf;
    
    xs = -10;
    ys = 0;
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








