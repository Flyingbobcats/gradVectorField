


%Plot activation functions

clc
clear 
close all
theta = 0:0.01:2*pi;


for i =1:length(theta)
R(i) = activationFunctions(theta(i),'r');
end

for i =1:length(theta)
A(i) = activationFunctions(theta(i),'a');
end

for i=1:length(theta)
   G(i) = activationFunctions(theta(i),'g');
end

subplot(3,1,1)
plot(rad2deg(theta),R);
ylabel('Repulsive Weight');

subplot(3,1,2)
plot(rad2deg(theta),A);
ylabel('Attractive Weight');


subplot(3,1,3)
plot(rad2deg(theta),G);
ylabel('Repulsion Coefficient');


