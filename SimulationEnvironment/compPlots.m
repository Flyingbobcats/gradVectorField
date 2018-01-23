%=========================================================================
% Plots for circular and line vector fields. Individual components and
% summs
%==========================================================================

clc
clear
close all

%Circular Field
circular =  false;
normsOffCirc = false;       %Will deactive ALL normilization


%Straight path
straight = false;
normsOffLine = false;

%Plot Decay
pltDecay = true;

if circular
    %Setup Field
    vf = vectorField;
    vf = vf.navf('circ');
    vf =  vf.xydomain(10,0,0,20);
    vf.avf{1}.r = 0.5;
    
    if normsOffCirc
        vf.avf{1}.normComponents = false;
        vf.avf{1}.normTotal      = false;
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
    vf.pltff
    vf.avf{1}.pltfnc
    %     title('Attractive');
    xlabel('x');
    ylabel('y');
    grid on
    axis([-10,10,-10,10]);
    set(gcf, 'PaperPosition', [0 0 5 5]); %Position plot at left hand corner with width 5 and height 5.
    set(gcf, 'PaperSize', [5 5]); %Set the paper to have width 5 and height 5.
    saveas(gcf, 'circAttractive', 'pdf') %Save figure
    
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
    if normsOffLine
        vf.avf{1}.normComponents = false;
        vf.avf{1}.normTotal = false;
        vf.normAttractiveFields = false;
        vf.NormSummedFields     = false;
        vf.normAttractiveFields = false;
        vf.normRepulsiveFields  = false;
    end
    
    vf = vectorField;
    vf = vf.navf('line');
    vf.avf{1}.angle = pi/2;
    vf.avf{1}.LineDistance = 20;
    vf =  vf.xydomain(10,0,0,20);
    
    
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
    
    
    %Negative Circulation
    figure
    vf.avf{1}.G = 1;
    vf.avf{1}.H = 1;
    
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
    saveas(gcf, 'lineConvCirc', 'pdf') %Save figure
    
    
end



if pltDecay
    
    vf = vectorField;
    vf = vf.nrvf('circ');
    vf.rvf{1} = vf.rvf{1}.modDecay('hyper');
    vf.rvf{1}.r = 0.5;
    vf.rvf{1}.decayR = 7;
    
    
    vf.NormSummedFields = false;
    
    
    hold on
    vf.pltff
    p1 = vf.rvf{1}.pltDecay;
    vf.rvf{1}.pltfnc
    xlabel('x');
    ylabel('y');
    grid on
    
    leg = legend([p1],{'Decay Radius R = 7'});
    set(leg,'fontsize',12);
    
    axis([-10,10,-10,10]);
    set(gca,'ytick',-10:5:10)
    set(gca,'xtick',-10:5:10)
    set(gcf, 'PaperPosition', [0 0 5 5]); %Position plot at left hand corner with width 5 and height 5.
    set(gcf, 'PaperSize', [5 5]); %Set the paper to have width 5 and height 5.
    saveas(gcf, 'circRepulsiveDecay', 'pdf') %Save figure
    
    
    
end
















