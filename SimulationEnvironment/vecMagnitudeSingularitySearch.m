%==========================================================================
%vecMagnitudeSingularitySearch.m
%
%
%=========================================================================


clc
clear
close all

vf = vectorField;

%Vector field converge to straight line path
vf = vf.navf('line');
vf.avf{1}.angle = pi/2;


%Repulsive circular vector field placed at origin with hyperbolic decay
vf = vf.nrvf('circ');
vf.rvf{1}.r = 0.01;
vf.rvf{1} = vf.rvf{1}.modDecay('hyper');

vf.rvfWeight = 1;
vf = vf.xydomain(20,0,0,250);


%Turn off normalization at all levels
vf.NormSummedFields = false;
vf.normAttractiveFields = false;
vf.normRepulsiveFields = false;
vf.avf{1}.normComponents = true;


vf.rvf{1}.normComponents = true;
vf.rvf{1}.normTotal = false;



func = @(x,y) vf.singularityDetect(x);

options = optimset('PlotFcns',@optimplotfval,'TolFun',1e-2);


xs = -5:5:5;
ys = xs;






Ss = [];
Ms = [];

runs = length(xs)*length(ys);
for i =1:length(xs)
    for j=1:length(ys)
        
        
    [solution,mag] = fminsearch(func,[xs(i),ys(j)],options);
    Ss = [Ss;solution];
    Ms = [Ms;mag];

    
    
    end
    
    str = strcat(num2str(i*j/runs*100), ' % complete.');
    disp(str);
end

figure
hold on
[X,Y,Ut,Vt] = vf.sumFields();
Z = sqrt(Ut.^2+Vt.^2);
p1 = surf(X,Y,Z);
p1 = plot3(Ss(:,1),Ss(:,2),Ms,'r*');
shading interp














