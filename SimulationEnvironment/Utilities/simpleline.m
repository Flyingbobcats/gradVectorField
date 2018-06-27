%=========================================================================
% simpleLine.m
%
% Description:
% Generate GVF for converging and following a line path, used in thesis
% defense presentation
%                                                     Author: Garrett Clem
%==========================================================================


clc
clear
close all

vf = vectorField;
vf = vf.navf('line');


vf = vf.nrvf('circ');
vf.rvfWeight = 0;
vf.rvf{1}.decayR = 4;
vf.rvf{1}.y = -5;
vf = vf.xydomain(20,0,0,25);

wpt = [0,-20;0,20];
hold on
vf.pltff

plot(wpt(:,1),wpt(:,2),'k','linewidth',3)
plot(wpt(:,1),wpt(:,2),'kx','markersize',20,'linewidth',5);
vf.pltDecay

axis([-10,10,-22,22]);
xlabel('x');
ylabel('y');
