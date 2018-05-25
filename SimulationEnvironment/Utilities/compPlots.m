%=========================================================================
% Plots for circular and line vector fields. Individual components and
% summs
%==========================================================================

clc
clear
close all


summedFieldsPlots = false;

%Circular Field
circular =  false;
normsOffCirc = false;       %Will deactive ALL normilization


%Straight path
straight = false;
normsOffLine = true;

%Plot Decay
pltDecay = true;

if circular
    %Setup Field
    vf = vectorField;
    vf = vf.navf('circ');
    vf =  vf.xydomain(50,0,0,15);
    vf.avf{1}.r = 35;
    
    vf.avf{1}.decayActive = false;
    
    if normsOffCirc
        vf.avf{1}.normComponents = false;
        vf.avf{1}.normTotal      = true;
        vf.normAttractiveFields = false;
        vf.NormSummedFields     = false;
        vf.normAttractiveFields = false;
        vf.normRepulsiveFields  = false;
    end
    
    %Attractive
    figure
    vf.avf{1}.G = 1;
    vf.avf{1}.H = 0;
    vf.avf{1}.L = 0;
    
    hold on
    set(gca,'fontsize',12);
    vf.pltff
    vf.avf{1}.pltfnc
    xlabel('x');
    ylabel('y');
    grid on
    axis([-50,50,-50,50]);
    set(gca,'ytick',-50:10:50)
    set(gca,'xtick',-50:10:50)
    set(gcf, 'PaperPosition', [0 0 5 5]); %Position plot at left hand corner with width 5 and height 5.
    set(gcf, 'PaperSize', [5 5]); %Set the paper to have width 5 and height 5.
    legend({'Guidance','Path'});
    
    %Repulsive
    figure
    vf.avf{1}.G = -1;
    vf.avf{1}.H = 0;
    vf.avf{1}.L = 0;
    
    hold on
    vf.pltff
    vf.avf{1}.pltfnc
    %     title('Repulsive');
    xlabel('x');
    ylabel('y');
    grid on
    axis([-10,10,-10,10]);
    set(gcf, 'PaperPosition', [0 0 5 5]); %Position plot at left hand corner with width 5 and height 5.
    set(gcf, 'PaperSize', [5 5]); %Set the paper to have width 5 and height 5.
    saveas(gcf, 'circRepulsive', 'pdf') %Save figure
    
    %Clockwise Circulation
    figure
    vf.avf{1}.G = 0;
    vf.avf{1}.H = 1;
    vf.avf{1}.L = 0;
    
    hold on
    vf.pltff
    vf.avf{1}.pltfnc
    %     title('CW Circulation');
    xlabel('x');
    ylabel('y');
    grid on
    axis([-10,10,-10,10]);
    set(gcf, 'PaperPosition', [0 0 5 5]); %Position plot at left hand corner with width 5 and height 5.
    set(gcf, 'PaperSize', [5 5]); %Set the paper to have width 5 and height 5.
    saveas(gcf, 'circCW', 'pdf') %Save figure
    
    %Counter Clockwise Circulation
    figure
    vf.avf{1}.G = 0;
    vf.avf{1}.H = -1;
    vf.avf{1}.L = 0;
    
    hold on
    vf.pltff
    vf.avf{1}.pltfnc
    %     title('CCW Circulation');
    xlabel('x');
    ylabel('y');
    grid on
    axis([-10,10,-10,10]);
    set(gcf, 'PaperPosition', [0 0 5 5]); %Position plot at left hand corner with width 5 and height 5.
    set(gcf, 'PaperSize', [5 5]); %Set the paper to have width 5 and height 5.
    saveas(gcf, 'circCCW', 'pdf') %Save figure
    
    %Time Varying
    figure
    vf.avf{1}.G = 0;
    vf.avf{1}.H = 0;
    vf.avf{1}.L = 1;
    vf.avf{1}.vx = 1;
    
    hold on
    vf.pltff
    vf.avf{1}.pltfnc
    %     title('Time Varying');
    xlabel('x');
    ylabel('y');
    grid on
    axis([-10,10,-10,10]);
    set(gcf, 'PaperPosition', [0 0 5 5]); %Position plot at left hand corner with width 5 and height 5.
    set(gcf, 'PaperSize', [5 5]); %Set the paper to have width 5 and height 5.
    saveas(gcf, 'circTv', 'pdf') %Save figure
    
    
    %Summed Convergence and Circulation
    figure
    vf.avf{1}.G = 1;
    vf.avf{1}.H = 1;
    vf.avf{1}.L = 0;
    
    hold on
    vf.pltff
    vf.avf{1}.pltfnc
    %     title('Convergence and Circulation');
    xlabel('x');
    ylabel('y');
    grid on
    axis([-10,10,-10,10]);
    set(gcf, 'PaperPosition', [0 0 5 5]); %Position plot at left hand corner with width 5 and height 5.
    set(gcf, 'PaperSize', [5 5]); %Set the paper to have width 5 and height 5.
    saveas(gcf, 'circConvCirc', 'pdf') %Save figure
    
    %Summed Convergence  Circulation and Time Varying
    figure
    vf.avf{1}.G = 1;
    vf.avf{1}.H = 1;
    vf.avf{1}.L = 1;
    
    hold on
    vf.pltff
    vf.avf{1}.pltfnc
    %     title('Convergence, Circulation, and Time Varying');
    xlabel('x');
    ylabel('y');
    grid on
    axis([-10,10,-10,10]);
    set(gcf, 'PaperPosition', [0 0 5 5]); %Position plot at left hand corner with width 5 and height 5.
    set(gcf, 'PaperSize', [5 5]); %Set the paper to have width 5 and height 5.
    saveas(gcf, 'circConvCircTv', 'pdf') %Save figure
    
    
    
    
end





if straight
    
    
    vf = vectorField;
    vf = vf.navf('line');
    vf.avf{1}.angle = pi/2;
    vf.avf{1}.LineDistance = 20;
    vf =  vf.xydomain(10,0,0,10);
    
    if normsOffLine
        vf.avf{1}.normComponents = false;
        %         vf.avf{1}.normTotal = false;
        vf.normAttractiveFields = false;
        vf.NormSummedFields     = false;
        vf.normAttractiveFields = false;
        vf.normRepulsiveFields  = false;
    end
    
    
    %Attractive
    figure
    vf.avf{1}.G = 1;
    vf.avf{1}.H = 0;
    
    hold on
    vf.pltff
    vf.avf{1}.pltfnc
    %     title('Attractive');
    xlabel('x');
    ylabel('y');
    grid on
    axis([-10,10,-10,10]);
    set(gcf, 'PaperPosition', [0 0 5 5]); %Position plot at left hand corner with width 5 and height 5.
    set(gcf, 'PaperSize', [5 5]); %Set the paper to have width 5 and height 5.
    saveas(gcf, 'lineAttractive', 'pdf') %Save figure
    
    %Repulsive
    figure
    vf.avf{1}.G = -1;
    vf.avf{1}.H = 0;
    
    hold on
    vf.pltff
    vf.avf{1}.pltfnc
    %     title('Repulsive');
    xlabel('x');
    ylabel('y');
    grid on
    axis([-10,10,-10,10]);
    set(gcf, 'PaperPosition', [0 0 5 5]); %Position plot at left hand corner with width 5 and height 5.
    set(gcf, 'PaperSize', [5 5]); %Set the paper to have width 5 and height 5.
    saveas(gcf, 'lineRepulsive', 'pdf') %Save figure
    
    %Circulation
    figure
    vf.avf{1}.G = 0;
    vf.avf{1}.H = 1;
    
    hold on
    vf.pltff
    vf.avf{1}.pltfnc
    %     title('Circulation');
    xlabel('x');
    ylabel('y');
    grid on
    axis([-10,10,-10,10]);
    set(gcf, 'PaperPosition', [0 0 5 5]); %Position plot at left hand corner with width 5 and height 5.
    set(gcf, 'PaperSize', [5 5]); %Set the paper to have width 5 and height 5.
    saveas(gcf, 'lineCirc', 'pdf') %Save figure
    
    %Negative Circulation
    figure
    vf.avf{1}.G = 0;
    vf.avf{1}.H = -1;
    
    hold on
    vf.pltff
    vf.avf{1}.pltfnc
    %     title('Negative Circulation');
    xlabel('x');
    ylabel('y');
    grid on
    axis([-10,10,-10,10]);
    set(gcf, 'PaperPosition', [0 0 5 5]); %Position plot at left hand corner with width 5 and height 5.
    set(gcf, 'PaperSize', [5 5]); %Set the paper to have width 5 and height 5.
    saveas(gcf, 'lineNegCirc', 'pdf') %Save figure
    
    
    %Combined
    figure
    vf.avf{1}.G = 1;
    vf.avf{1}.H = 1;
    
    hold on
    set(gca,'fontsize',12);
    vf.pltff
    vf.avf{1}.pltfnc
    %         title('Convergence and Circulation');
    xlabel('x');
    ylabel('y');
    grid on
    axis([-10,10,-10,10]);
    set(gcf, 'PaperPosition', [0 0 5 5]); %Position plot at left hand corner with width 5 and height 5.
    set(gcf, 'PaperSize', [5 5]); %Set the paper to have width 5 and height 5.
    legend({'Guidance','Path'});
    
    
    
    %Combined
    figure
    vf.avf{1}.G = 1;
    vf.avf{1}.H = 5;
    
    hold on
    set(gca,'fontsize',12);
    vf.pltff
    vf.avf{1}.pltfnc
    %         title('Convergence and Circulation');
    xlabel('x');
    ylabel('y');
    grid on
    axis([-10,10,-10,10]);
    set(gcf, 'PaperPosition', [0 0 5 5]); %Position plot at left hand corner with width 5 and height 5.
    set(gcf, 'PaperSize', [5 5]); %Set the paper to have width 5 and height 5.
    
    legend({'Guidance','Path'});
    
    
end



if pltDecay
    
    vf = vectorField;
    vf = vf.xydomain(50,0,0,20);
    vf = vf.nrvf('circ');
    vf.rvf{1} = vf.rvf{1}.modDecay('hyper');
    vf.rvf{1}.r = 0.5;
    vf.rvf{1}.decayR = 35;
    
    
    vf.NormSummedFields = false;
    
    
    hold on
    vf.pltff
    p1 = vf.rvf{1}.pltDecay;
    vf.rvf{1}.pltfnc
    set(gca,'fontsize',12);
    
    xlabel('x');
    ylabel('y');
    grid on
    
    %     leg = legend([p1],{'Decay Radius R = 35'});
    leg = legend({'Guidance','Decay Radius','Obstacle Center'});
    
    set(leg,'fontsize',12);
    
    axis([-50,50,-50,50]);
    set(gca,'ytick',-50:10:50)
    set(gca,'xtick',-50:10:50)
    set(gcf, 'PaperPosition', [0 0 5 5]); %Position plot at left hand corner with width 5 and height 5.
    set(gcf, 'PaperSize', [5 5]); %Set the paper to have width 5 and height 5.
    saveas(gcf, 'circRepulsiveDecay', 'pdf') %Save figure
    
    
    
    figure
    vf = vectorField;
    vf = vf.xydomain(50,0,0,20);
    
    vf = vf.nrvf('circ');
    vf.rvf{1} = vf.rvf{1}.modDecay('hyper');
    vf.rvf{1}.H = 1;
    vf.rvf{1}.r = 0.5;
    vf.rvf{1}.decayR = 35;
    
    
    vf.NormSummedFields = false;
    
    
    hold on
    vf.pltff
    p1 = vf.rvf{1}.pltDecay;
    vf.rvf{1}.pltfnc
    set(gca,'fontsize',12);
    xlabel('x');
    ylabel('y');
    grid on
    
    leg = legend({'Guidance','Decay Radius','Obstacle Center'});
    set(leg,'fontsize',12);
    
    axis([-50,50,-50,50]);
    set(gca,'ytick',-50:10:50)
    set(gca,'xtick',-50:10:50)
    set(gcf, 'PaperPosition', [0 0 5 5]); %Position plot at left hand corner with width 5 and height 5.
    set(gcf, 'PaperSize', [5 5]); %Set the paper to have width 5 and height 5.
    saveas(gcf, 'circRepulsiveDecay', 'pdf') %Save figure
    
    
    
end





if summedFieldsPlots
    
    
    vf = vectorField;
    
    
    vf = vf.navf('line');
    vf.avf{1}.angle = pi/2;
    vf.avf{1}.LineDistance = 20;
    vf.avf{1}.H = 5;
    
    vf.avf{1}.normComponents = false;
    vf.normAttractiveFields = false;
    vf.NormSummedFields     = false;
    vf.normAttractiveFields = false;
    vf.normRepulsiveFields  = false;
    
    vf =  vf.xydomain(50,0,0,25);
    vf = vf.nrvf('circ');
    vf.rvf{1}.H=1;
    vf.rvf{1}.decayR = 35;
    vf.rvf{1} = vf.rvf{1}.modDecay('hyper');
    vf.rvf{1}.r = 0.01;
    vf.rvf{1}.decayR = 35;
    
    vf.NormSummedFields = true;
    hold on
    
%             plot([-50,50],[0,0],'k','linewidth',2,'alpha',1);
    vf.pltff();

    vf.rvf{1}.pltEqualStrength;
    axis([-50,50,-25,25]);
    set(gca,'ytick',-50:10:50)
    set(gca,'xtick',-50:10:50)
    set(gca,'fontsize',12);
    xlabel('x');
    ylabel('y');
    
    legend({'Guidance','Equal Strength','Obstacle Center'});
    
end
    
%     vf =  vf.xydomain(50,0,0,200);
%     vf.NormSummedFields = false;
%     [x,y,u,v] = vf.sumFields();
%     for i=1:length(x)
%         for j = 1:length(y)
%             
%             mag(i,j) = sqrt(u(i,j)^2+v(i,j)^2);
%         end
%     end
%     
%     figure
%     surf(x,y,mag)
%     set(gca,'fontsize',12);
%     shading interp
%     view([0,90]);
%     axis equal
%     axis([-50,50,-25,25]);
%     xlabel('x');
%     ylabel('y');
%     h = colorbar();
%     ylabel(h,'Vector Magnitude');
%     grid on
%     
%     
%     figure
%     [C,h] = contour(x,y,mag,20);
%     
%     v = linspace(0,0.1,20);
%     
%     clabel(C,h,v)
%     set(gca,'fontsize',12);
%     shading interp
%     view([0,90]);
%     axis equal
%     axis([-50,50,-25,25]);
%     xlabel('x');
%     ylabel('y');
%     
%     
%     grid on
%     
%     
%     
%     
% end

%
%
%
%     vf.pltff
%     vf.avf{1}.pltfnc
%     %     title('Repulsive');
%     xlabel('x');
%     ylabel('y');
%     grid on
%     axis([-10,10,-10,10]);
%     set(gcf, 'PaperPosition', [0 0 5 5]); %Position plot at left hand corner with width 5 and height 5.
%     set(gcf, 'PaperSize', [5 5]); %Set the paper to have width 5 and height 5.
%     saveas(gcf, 'circRepulsive', 'pdf') %Save figure

%
%     close all
%     figure
%     r = 5;
%     theta = 0:0.1:2.1*pi;
%     xs = r*cos(theta);
%     ys = r*sin(theta);
%
%     vf = vectorField();
%     vf = vf.navf('circ');
%     vf.avf{1}.G = 1;
%     vf.avf{1}.H = 1;
%     vf.avf{1}.L = 0;
%
%     vf =  vf.xydomain(6,0,0,35);
%     vf.avf{1}.r = r;
%
%     hold on
%     vf.pltff()
%     plot(xs,ys,'r','linewidth',3);
%
%     set(gca,'fontsize',12);
%     legend({'Guidance','Path to leader craft'});



%      vf = vectorField;
%
%
%     vf = vf.navf('cosi');
%         vf =  vf.xydomain(10,0,0,30);
%     vf.avf{1}.angle = pi/2;
%     vf.avf{1}.LineDistance = 20;
%     vf.avf{1}.H = 2;
%     vf.avf{1}.amplitude = 3;
%     vf.avf{1}.phase = 7;
%
%     vf.avf{1}.normComponents = false;
%     vf.normAttractiveFields = false;
%     vf.NormSummedFields     = false;
%     vf.normAttractiveFields = false;
%     vf.normRepulsiveFields  = false;
%
%     figure
%     hold on
%     vf.pltff();
%     plot(-10:0.01:10, vf.avf{1}.amplitude*sin(-10:0.01:10),'r','linewidth',2);
%     axis([-10,10,-6,6]);
%     axis equal
%     set(gca,'fontsize',12);
%     xlabel('x');
%     ylabel('y');
%     legend({'Guidance','Path'});










