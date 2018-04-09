%=========================================================================
% numericallySolveGammaAndH.m
%
%
%==========================================================================



clc
clear
close all

v = 100;
obstR = v/0.35+1000;
obstY = 0;
tr = v/0.35;
lbGamma = (obstR / (tr));
dt = 0.1;

plotFinal = false;

fr = @(X) VF(X,v,dt,plotFinal,obstR,obstY);         %Find min with respect to r



options = optimoptions('fmincon','Display','final-detailed');
options.DiffMinChange = 0.05;
options.DiffMaxChange = 1;
options.PlotFcn = @optimplotfval;
options.StepTolerance = 1e-5;

x0 = [lbGamma,2];
A = [];
b = [];
Aeq = [];
beq = [];
lb = [lbGamma,1];
ub = [lbGamma*1.1,6];



tic
[Xsolved,costR] = fmincon(fr,x0,A,b,Aeq,beq,lb,ub,[],options);
sim_time = toc;

% Xsolved = [3.4,3];

plotFinal = true;
figure
pause()
fr = @(X) VF(X,v,dt,plotFinal,obstR,obstY);
cost = fr(Xsolved);
disp(Xsolved)
disp(cost)
str = strcat('Minimizing Cost Avoidance with Circulation ','\gamma = ',num2str(Xsolved(1)),{' '}, 'h=',num2str(Xsolved(2)),{' '},'cost=',num2str(cost),{' '},'time=',num2str(sim_time));
title(str)
xlabel('x');
ylabel('y');



% 
% plotFinal = true;
% figure
% pause()
% fr = @(X) VF(X,v,dt,plotFinal,obstR,obstY);
% Xsolved = [2.5,4];
% cost = fr(Xsolved);
% str = strcat('No Circulation Avoidance','\gamma = ',num2str(Xsolved(1)),{' '}, 'h=',num2str(Xsolved(2)),{' '},'cost=',num2str(cost),{' '});
% title(str)
% 
% xlabel('x');
% ylabel('y');






function cost = VF(X,velocity,dt,plotFinal,obstR,obstY)

    gamma = X(1);
    H = X(2);
%     G = X(3);
    
    
       G = -1;
    
    plotFlight = false;

    %Setup vector field
        vf = vectorField();
        

        
        %Goal Path
        vf = vf.navf('line');
        vf.avf{1}.angle = pi/2;
        vf.NormSummedFields = false;
        vf.avf{1}.H = velocity;
        vf.avf{1}.normComponents = false;
        vf.normAttractiveFields = false;

        
        %Obstacle
        vf = vf.nrvf('circ');
        vf.rvf{1}.r = 0.01;
        vf.rvf{1}.H = H;
        vf.rvf{1}.G = G;
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
        
        

        
        %Change UAVs starting position to just outside the decay field edge

        
        %Set decay field strength based on gamma ratio, plus obstacles
        %radius
        
        vf.rvf{1}.decayR = uav.turn_radius*gamma+obstR;
        
        vf.rvf{1} = vf.rvf{1}.modDecay('hyper');
        
        %Cost function initially 0
        cost = 0;

    cost = 0;
    
    RANGE = [];
    AVFW = [];
    RVFW = [];
    BETA = [];
    ALPHA = [];
    GS = [];
    
    while uav.x<=(uav.turn_radius*gamma+obstR)*1.1
        
        
        
        alpha = atan2(uav.y-obstY,uav.x);
        range = sqrt((uav.x-vf.rvf{1}.x)^2+(uav.y-vf.rvf{1}.y)^2);
        beta = pi - alpha+uav.heading;
        
        
                        
                turnR = uav.turn_radius;
                obstX = 0;
                y = turnR*(1-cos(pi/2));
                X = obstX - sqrt((turnR+obstR)^2-(y-obstY)^2);
        if abs(uav.x)<= abs(X)
        
        vf.rvfWeight = 2*activationFunctions(alpha,'r');
        vf.avfWeight = 1/4*activationFunctions(alpha,'a');
        
        

        

        g = -activationFunctions(abs(beta),'g');
        vf.rvf{1}.G = 2*g;
        

%         vf.avf{1}.H = obstR/2+1/abs(uav.y+0.5);


        
        
        turnR = uav.turn_radius;
        y = turnR*(1-cos(pi/2));
        X = 0 - sqrt((turnR+obstR)^2-(y-obstY)^2);
        
        
        theta = asin((y-obstY)/(obstR+turnR));
        zeta = pi+theta;
        
        X_turn = -1*(X+turnR*cos(zeta));
        Y_turn = y+turnR*sin(zeta);
        
        turn_at_angle = atan2(Y_turn,X_turn);
        
%         if alpha <= turn_at_angle
%             vf.avf{1}.H = obstR/abs(uav.y);
%         end

        if uav.x>=abs(X)-uav.turn_radius
            vf.avfWeight = 1;
            vf.rvfWeight = 0;
        end
        else
            vf.avfWeight = 1;
            vf.rvfWeight = 0;
        end
        
        ALPHA = [ALPHA,alpha];
        BETA = [BETA,beta];
        RVFW = [RVFW,vf.rvfWeight];
        AVFW = [AVFW,vf.avfWeight];
        GS = [GS,vf.rvf{1}.G];
        

       

        [u,v]=vf.heading(uav.x,uav.y);
        heading_cmd = atan2(v,u);

        if uav.heading > deg2rad(175) && uav.heading <deg2rad(285)
            cost = 1000;
            break
        end
        uav = uav.update_pos(heading_cmd);


        
%         
%         if abs(uav.x)>obstR
%             cost = cost+ abs(uav.y) / ((vf.rvf{1}.decayR))*dt;
%         else
%             cost = cost+abs(range-obstR)/(vf.rvf{1}.decayR)*dt;
%         end
%         
%         if range < obstR
%             cost = cost+100;
%         end
            

        cost = cost+ abs(uav.y) / ((vf.rvf{1}.decayR))*dt;

        
        if range < obstR
            cost = cost+100;
        end
%         
        
        
           if plotFlight == true && plotFinal == true
               
                       vf = vf.xydomain((uav.turn_radius*gamma+obstR)*1.1,0,0,10);
            
            clf
            subplot(5,7,[6,7]);
            plot(rad2deg(ALPHA),'r-');
            ylabel('\alpha (deg)');
            grid on
           
            
            subplot(5,7,[13,14]);
            plot(rad2deg(BETA),'r-');
            ylabel('\beta (deg)');
            grid on
            
            subplot(5,7,[20,21]);
            plot(RVFW,'r');            
            ylabel('RVFW');
            grid on
            
            subplot(5,7,[27,28]);
            plot(AVFW,'r');
            ylabel('AVFW');
            
            subplot(5,7,[34,35]);
            plot(GS,'r');
            ylabel('G');
            
            subplot(5,7,[1:5,8:12,15:19,22:26,29:33]);
                                       optPath = genOptPath(uav,obstR);
               plot(optPath(:,1),optPath(:,2),'b.');
            title(num2str(cost));

            
            vf.NormSummedFields = true;
            uav.colorMarker = 'k-';

         
            hold on
            p1 = vf.pltff();
            

            
            
            vf.rvf{1}.pltDecay();
            
   
            
            options = optimoptions('fsolve','Display','off','Algorithm','levenberg-marquardt');%,'UseParallel',true);
            ops.m = 10;
            ops.n = 10;
            ops.xlimit = 10;
            ops.ylimit = 10;
            ops.r = obstR;
            ops.d_theta = deg2rad(10);
            XYS = icPoints('circle',ops);
            
            fun = @(X) magVF(X,vf);
            location = cell(1,length(XYS));
            gradMag  = cell(1,length(XYS));
            solverFlag = cell(1,length(XYS));
            
            
            

            
%             for i =1:length(XYS)
%                 X0 = [XYS(1,i),XYS(2,i)];
%                 [location{i},gradMag{i},solverFlag{i}] = fsolve(fun,X0,options);
%             end
%             
%             for i =1:length(XYS)
%                 x0 = XYS(1,i);
%                 y0 = XYS(2,i);
%                 x = location{i};
%                 if solverFlag{i} == -2
%                 elseif solverFlag{i} ==1 || solverFlag{i} ==2 || solverFlag{i} ==3 || solverFlag{i} ==4
% %                     p4 = plot(x0,y0,'ko','markersize',7);
%                     p7 = plot(x(1),x(2),'ro','markersize',10,'markerfacecolor','r');
% %                     p6 =plot([x(1),x0],[x(2),y0],'k--','markersize',15);
%                 end
%                 
%                
%             end
                
                p2 = plot(uav.xs(1),uav.ys(1),'db','markersize',10,'markerfacecolor','b');
                p3 =plot(uav.xs(end),uav.ys(end),'dr','markersize',10,'markerfacecolor','r');
                                

                p5 =plot([uav.xs(1),uav.xs(end)],[0,0],'g','linewidth',3);
                p6 =plot(obstx,obsty,'r','linewidth',2);                                     
%                 p7 = plot(x(1,1),x(1,2),'ro','markersize',5,'markerfacecolor','r');
                
                axis([-(uav.turn_radius*gamma+obstR)*1.1,(uav.turn_radius*gamma+obstR)*1.1,-(uav.turn_radius*gamma+obstR)*1.1,(uav.turn_radius*gamma+obstR)*1.1]);
                p4 = uav.pltUAV();

                axis equal
                grid on
                
%                 legend([p1,p2,p3,p4,p5,p6,p7],{'Summed Guidance','UAV Start','UAV End','UAV Path','Planned Path','Obstacle','Singularity'});%,'Obstacle','UAV Path','Singularity'});
            
            drawnow();
%                pause();
                
%                 figure
%                 subplot(3,1,1);
%                 plot(uav.ts(2:end),AVFW);
%                 ylabel('AVFW');
%                 
%                 subplot(3,1,2);
%                 plot(RVFW);
%                 ylabel('RVFW');
%                 
%                 subplot(3,1,3);
%                 plot(RANGE);
%                 ylabel('RANGE');
%         end
        

                end
    end
    
    
    
        if plotFinal == true
            
            vf.NormSummedFields = true;
            uav.colorMarker = 'b-';

            hold on
                           optPath = genOptPath(uav,obstR);
               plot(optPath(:,1),optPath(:,2),'k.');
               
            p1 = vf.pltff();
            vf.rvf{1}.pltDecay();
            
            options = optimoptions('fsolve','Display','off','Algorithm','levenberg-marquardt');%,'UseParallel',true);
            ops.m = 10;
            ops.n = 10;
            ops.xlimit = 10;
            ops.ylimit = 10;
            ops.r = obstR;
            ops.d_theta = deg2rad(10);
            XYS = icPoints('circle',ops);
            
            fun = @(X) magVF(X,vf);
            location = cell(1,length(XYS));
            gradMag  = cell(1,length(XYS));
            solverFlag = cell(1,length(XYS));
            
            
            

            
            for i =1:length(XYS)
                X0 = [XYS(1,i),XYS(2,i)];
                [location{i},gradMag{i},solverFlag{i}] = fsolve(fun,X0,options);
            end
            
            for i =1:length(XYS)
                x0 = XYS(1,i);
                y0 = XYS(2,i);
                x = location{i};
                if solverFlag{i} == -2
                elseif solverFlag{i} ==1 || solverFlag{i} ==2 || solverFlag{i} ==3 || solverFlag{i} ==4
%                     p4 = plot(x0,y0,'ko','markersize',7);
                    p7 = plot(x(1),x(2),'ro','markersize',10,'markerfacecolor','r');
%                     p6 =plot([x(1),x0],[x(2),y0],'k--','markersize',15);
                end
                
               
            end
                
                p2 = plot(uav.xs(1),uav.ys(1),'db','markersize',10,'markerfacecolor','b');
                p3 =plot(uav.xs(end),uav.ys(end),'dr','markersize',10,'markerfacecolor','r');
                                

                p5 =plot([uav.xs(1),uav.xs(end)],[0,0],'g','linewidth',3);
                p6 =plot(obstx,obsty,'r','linewidth',2);                                     
                p7 = plot(x(1,1),x(1,2),'ro','markersize',5,'markerfacecolor','r');
                
                
                                                p4 = uav.pltUAV();

                axis equal
                grid on
                                axis([-(uav.turn_radius*gamma+obstR)*1.1,(uav.turn_radius*gamma+obstR)*1.1,-(uav.turn_radius*gamma+obstR)*1.1,(uav.turn_radius*gamma+obstR)*1.1]);

                legend([p1,p2,p3,p4,p5,p6,p7],{'Summed Guidance','UAV Start','UAV End','UAV Path','Planned Path','Obstacle','Singularity'});%,'Obstacle','UAV Path','Singularity'});

                
                

        end
end



function F = magVF(X,vf)

    [U,V]=vf.heading(X(1),X(2));
    F(1) = U;
    F(2) = V;

end







