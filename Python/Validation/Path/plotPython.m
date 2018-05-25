

clc
clear
close all

path = false;
obst = false;

summed = true;
simSummed = true;






if path
    X = importfile('path\Xs',1,1000000);
    Y = importfile('path\Ys',1,1000000);
    U = importfile('path\Us',1,1000000);
    V = importfile('path\Vs',1,1000000);
    quiver(X,Y,U,V,'k','linewidth',3)
    vf = vectorField();
    vf = vf.navf('line');
    vf.avf{1}.angle = pi/2;
    vf.NormSummedFields = true;
    vf.avf{1}.H = 5;
    vf.avf{1}.normComponents = false;
    vf.normAttractiveFields = true;
    vf = vf.xydomain(50,0,0,2*50/3);
end

if obst
    X = importfile('obstacle\Xs',1,1000000);
    Y = importfile('obstacle\Ys',1,1000000);
    U = importfile('obstacle\Us',1,1000000);
    V = importfile('obstacle\Vs',1,1000000);
    quiver(X,Y,U,V,'k','linewidth',3)
    %Setup vector field
    vf = vectorField();
    vf = vf.xydomain(50,0,0,2*50/3);
 
    vf.NormSummedFields = false;
    vf.normAttractiveFields = false;
    
    %Obstacle
    vf = vf.nrvf('circ');
    vf.rvf{1}.r = 0.01;
    vf.rvf{1}.H = 1;
    vf.rvf{1}.G = -1;
    vf.rvf{1}.decayR = 30;
    vf.rvf{1}.decayActive = true;
end



if summed
    X = importfile('summed\Xs',1,1000000);
    Y = importfile('summed\Ys',1,1000000);
    U = importfile('summed\Us',1,1000000);
    V = importfile('summed\Vs',1,1000000);
%     quiver(X,Y,U,V,'k','linewidth',3)
    
    %Setup vector field
    vf = vectorField();
    vf = vf.xydomain(50,0,0,2*50/3);
    
    vf = vf.navf('line');
    vf.avf{1}.angle = pi/2;
    vf.NormSummedFields = true;
    vf.avf{1}.H = 5;
    vf.avf{1}.normComponents = false;
    vf.normAttractiveFields = true;
    vf = vf.xydomain(50,0,0,2*50/3);
 
    vf.NormSummedFields = true;
    vf.normAttractiveFields = false;
    
    %Obstacle
    vf = vf.nrvf('circ');
    vf.rvf{1}.r = 0.01;
    vf.rvf{1}.H = 1;
    vf.rvf{1}.G = -1;
    vf.rvf{1}.decayR = 30;
    vf.rvf{1}.decayActive = true;
    
    
    
    if simSummed

        
        uav = UAV();
                uav.plotHeading = false;
        uav.plotCmdHeading = false;
        
        uav.plotUAV = false;
        uav.plotUAVPath = true;
        uav.plotFlightEnv = false;
        uav = uav.setup(-40, 0, 1, 0, 0.1);
        while uav.x<50
            [u,v] = vf.heading(uav.x,uav.y);
            uav = uav.update_pos(atan2(v,u));
            
        end
        hold on
%         uav.colorMarker = 'r-';
        uav.pltUAV();
        
        pyUAVx = importfile('summed\UAVpathx',1,1000000);
        pyUAVy = importfile('summed\UAVpathy',1,1000000);
        plot(pyUAVx,pyUAVy,'k-','linewidth',2);
        axis equal
        legend({'MATLAB','Python'});
        grid on
        xlabel('x')
        ylabel('y')
    end
    
end


% 
% hold on
% vf.pltff()
% xlabel('x')
% ylabel('y')
% title('Validating python guidance');
% legend({'Python','MATLAB'});
% set(gca,'fontsize',12)




