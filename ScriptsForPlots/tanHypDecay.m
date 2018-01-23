%tan hyperbolic function


theta = 0:0.01:2*pi;

decay = tanh(-theta)/2+0.5;

figure
plot(theta,decay)