%Call IC

clc
clear
close all


options.m = 10;
options.n = 10;
options.xlimit = 10;
options.ylimit = 10;
options.r = 10;
options.d_theta = 0.010;
XYS = icPoints('circle');
grid on
axis equal