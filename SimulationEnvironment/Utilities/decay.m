
clc
clear
close all


r = 0:0.01:1;





R = 1;
p = -(tanh(2*pi*r/R-pi))+1;

plot(r,p,'linewidth',2)
xlabel('range/radius');
ylabel('decray strength');
grid on





syms r
s = solve(-(tanh(2*pi*r/2-pi))+1==1,r)


