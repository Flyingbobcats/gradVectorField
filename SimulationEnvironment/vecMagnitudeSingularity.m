%==========================================================================
%vecMagnitudeSingularity.m
%
% Script for plotting vector field magnitude as surface plot for a circular
% obstacle placed on a straight line path
%
%
%=========================================================================


clc
clear
close all

vf = vectorField;

%Vector field converge to straight line path
vf = vf.navf('line');
vf.avf{1}.angle = pi/2;


%Repulsive circular vector field placed at origin with hyperbolic decay
vf = vf.nrvf('circ');
vf.rvf{1}.r = 0.01;
vf.rvf{1} = vf.rvf{1}.modDecay('hyper');

vf.rvfWeight = 1;
vf = vf.xydomain(10,0,0,100);


%Turn off normalization at all levels
vf.NormSummedFields = false;
vf.normAttractiveFields = false;
vf.normRepulsiveFields = false;
vf.avf{1}.normComponents = true;


vf.rvf{1}.normComponents = true;
vf.rvf{1}.normTotal = false;

%Plotting results
hold on
vf.pltff                              %Plot entire field
% vf.rvf{1}.pltDecay                  %Plot repulsive field edge
% vf.rvf{1}.pltEqualStrength          %Plot radius of equal strengths

xlabel('x');
ylabel('y')
axis equal
title('Summed fields with obstacle and goal normalization');





figure
[X,Y,Ut,Vt] = vf.sumFields();
mag = sqrt(Ut.^2+Vt.^2);
surf(X,Y,mag);
xlabel('x');
ylabel('y')
title('Summed fields without normalization');




% vf.NormSummedFields = true;
% [X,Y,Ut,Vt] = vf.sumFields();
% mag = sqrt(Ut.^2+Vt.^2);
% figure
% surf(X,Y,mag);
% xlabel('x');
% ylabel('y')
% title('Summed fields with normalization');






