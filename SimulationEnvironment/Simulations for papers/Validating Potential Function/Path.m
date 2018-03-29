
clc
clear
close all
syms theta x y z

a1 = cos(theta)*x + sin(theta)*y;
a2 = z;

expand(a1^2+a2^2)

g1 = [cos(theta);sin(theta);0];
g2 = [0;0;1];

cross(g1,g2)