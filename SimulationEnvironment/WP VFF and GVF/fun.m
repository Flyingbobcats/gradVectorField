function F= fun(gamma,x1,y1,L,obstR,obstX,obstY,turnR,type)

if type =='g'
x2 = obstX + obstR*cos(gamma);
y2 = obstY + obstR*sin(gamma);
end

if type =='z'
    y = turnR;
X = obstX - sqrt((turnR+obstR)^2-(y-obstY)^2);
    x2 = -X+turnR*cos(gamma);
    y2 = y+turnR*sin(gamma);
end

l = sqrt((x1-x2)^2+(y1-y2)^2);

F = L - l;

end