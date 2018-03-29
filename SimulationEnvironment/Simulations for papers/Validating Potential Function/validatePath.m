

clc
clear
close all

xs= -10:1:10;
ys=xs;

z = 1;
theta = pi/2;


for i =1:length(xs)
    for  j=1:length(ys)
    
        A= -1 /(2*(cos(theta)^2*xs(i)^2+2*cos(theta)*sin(theta)*xs(i)*ys(j)+sin(theta)^2*ys(j)^2+z^2));
        A = -1;
        B = [ 2*xs(i)*cos(theta)^2+2*cos(theta)*sin(theta)*ys(j);
              2*ys(j)*sin(theta)^2+2*cos(theta)*sin(theta)*xs(i)
              2*z];
          
          
       Vconv = A*B;
       
       Vcirc = [sin(theta);-cos(theta);0];
       
       V =  Vconv+Vcirc;
       V = V/sqrt(V(1)^2+V(2)^2);
       
       X(i,j) = xs(i);
       Y(i,j) = ys(j);
       Us(i,j) = V(1);
       Vs(i,j) = V(2);
            
end
end

quiver(X,Y,Us,Vs,'r');
hold on


vf = vectorField();
vf = vectorField();

vf.xspace = xs;
vf.yspace = ys;



%Goal Path
vf = vf.navf('line');
vf.avf{1}.angle = pi/2;
vf.avf{1}.normComponents = false;

vf.pltff()
