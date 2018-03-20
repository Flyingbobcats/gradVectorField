%==========================================================================
%
%
% No solid definition of an obstacle has been made yet. The problem should
% be:
%
% Determine a decay radius that produces a guidance which avoids the
% obstacle region.
%
%
% The obstacle region is defined as a circular hull of radius (r). A decay
% radius (decayR) must be selected such that the UAV does not enter the
% hull. The radius of equal strength, where singularities may occur, is
% located at half the decay radius.
%
%==========================================================================


clc
clear
close all

vf = vectorField();

vf = vf.navf('line');
vf.avf{1}.angle = pi/2;
vf.NormSummedFields = 0;



%Define some arbirary keep out zone
r = 2;
cxs = r*cos(0:0.01:2*pi);
cys = r*sin(0:0.01:2*pi);

%Obstacle
vf = vf.nrvf('circ');
vf.rvf{1}.r = 0.01;
vf.rvf{1}.decayR = 8;
vf.rvf{1} = vf.rvf{1}.modDecay('hyper');

hold on
plot(cxs,cys,'r','linewidth',3)
vf.rvf{1}.pltDecay();
vf.rvf{1}.pltEqualStrength();
vf.pltff();


