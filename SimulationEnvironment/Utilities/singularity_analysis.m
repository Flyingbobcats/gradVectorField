
clc
clear
close all

%Straight path vector field
vf = vectorField();
vf = vf.navf('line');
vf.avf{1}.angle = pi/2;
vf.avfWeight = 1;
vf.avf{1}.G=1;



vf = vf.nrvf('circ');
vf.rvfWeight = 1;
vf.rvf{1}.decayR = 5;
vf.rvf{1}= vf.rvf{1}.modDecay('hyper');
vf.rvf{1}.r = 0.01;
vf.rvf{1}.y = 0;
vf = vf.xydomain(10,0,0,50);
vf.rvf{1}.H = 1;
vf.NormSummedFields = 1;


figure
hold on
vf.pltff
vf.rvf{1}.pltDecay()
vf.rvf{1}.pltEqualStrength();


