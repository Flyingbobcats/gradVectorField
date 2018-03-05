

clc
clear 
close all

vf = vectorField();

%Goal Path
vf = vf.navf('line');
vf.avf{1}.angle = pi/2;
vf.NormSummedFields = 0;


%Obstacle
vf = vf.nrvf('circ');
vf.rvf{1}.r = 0.01;
vf.rvf{1} = vf.rvf{1}.modDecay('hyper');
vf.rvf{1}.H = 0;
vf.rvf{1}.decayR = 10;

vf.pltff

r = 0:0.1:10;
decayR = 5;

F = @(r)-(tanh(2*pi*r/decayR-pi))+1;

figure
plot(r,F(r));