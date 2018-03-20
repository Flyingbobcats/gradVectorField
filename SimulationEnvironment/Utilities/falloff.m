

clc
clear
v = vectorField;
v = v.nrvf('circ');
v.rvf{1} = v.rvf{1}.modDecay('hyper');
v.NormSummedFields = 0;
v.rvf{1}.r = 0.1;
v.rvf{1}.decayR = 5;
v.pltff
close all
v.pltff
hold on
v.pltDecay

[x,y,u,v] = v.sumFields;

max = 0;
for i = 1:length(x)
    for j = 1:length(y)
   
        if norm([u(i,j),v(i,j)]) > max
            max = norm([u(i,j),v(i,j)]);
        end
        
        
    end
end