
clc
clear
close all

vf = vectorField;
vf = vf.navf('line');
vf.avf{1}.angle = pi/2;

vf = vf.nrvf('circ');
vf.rvf{1}.r = 0.01;
vf.rvf{1} = vf.rvf{1}.modDecay('hyper');

vf.rvfWeight = 10;
vf = vf.xydomain(10,0,0,60);

hold on
vf.pltff
vf.rvf{1}.pltDecay
