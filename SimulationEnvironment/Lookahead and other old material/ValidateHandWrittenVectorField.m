%Confirming straight path equations


clc
clear
close all

xs = -10:0.5:10;
ys = xs;

Us = [];
Vs = [];

theta = deg2rad(90);
a = cos(theta);
b = sin(theta);

for i=1:length(xs)
    for j = 1:length(ys)
        
        V = -[ (xs(i)*a+ys(j)*b)*a ; (xs(i)*a+ys(j)*b)*b ; 1] + [b;-a;0];
        
        mag = 1%sqrt(V(1)^2+V(2)^2);
        Us(i,j) = V(1)/mag;
        Vs(i,j)  = V(2)/mag;
        XS(i,j) = xs(i);
        YS(i,j) = ys(j);
        
    end
end




quiver(XS,YS,Us,Vs);
axis equal


yc = 0;
xc = 0;
r = 2;
for i=1:length(xs)
    for j = 1:length(ys)
        
        V = [ -2*(xs(i)-xc)*((xs(i)-xc)^2+(ys(j)-yc)^2-r^2)+2*(ys(j)-yc);
            -2*(ys(j)-yc)*((xs(i)-xc)^2+(ys(j)-yc)^2-r^2)-2*(xs(i)-xc);
            1];
            
        
        mag = 1%sqrt(V(1)^2+V(2)^2);
        Us(i,j) = V(1)/mag;
        Vs(i,j)  = V(2)/mag;
        XS(i,j) = xs(i);
        YS(i,j) = ys(j);
        
    end
end

figure
quiver(XS,YS,Us,Vs);
axis equal



