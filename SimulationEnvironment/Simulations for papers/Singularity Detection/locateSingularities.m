function sing = locateSingularities(vf,plotHeat,numericallyLocate,obstR)




if numericallyLocate
    %Circular field
    options = optimoptions('fsolve','Display','off','Algorithm','levenberg-marquardt');%,'UseParallel',true);
    R = vf.rvf{1}.decayR/2;
    
    ops.m = 10;
    ops.n = 10;
    ops.xlimit = 10;
    ops.ylimit = 10;
    ops.r = R;
    ops.d_theta = deg2rad(10);
    XYS = icPoints('circle',ops);
    
    fun = @(X) VF(X,vf);
    location = cell(1,length(XYS));
    gradMag  = cell(1,length(XYS));
    solverFlag = cell(1,length(XYS));
    
    vf.NormSummedFields = false;
    
    
   
    for i =1:length(XYS)
        X0 = [XYS(1,i),XYS(2,i)];
        [location{i},gradMag{i},solverFlag{i}] = fsolve(fun,X0,options);
    end
    
    
    sing = [];
    for i =1:length(XYS)
        x0 = XYS(1,i);
        y0 = XYS(2,i);
        x = location{i};
        if solverFlag{i} == -2
            %plot nothing, this was from older versions
        elseif solverFlag{i} ==1 || solverFlag{i} ==2 || solverFlag{i} ==3 || solverFlag{i} ==4
%             p6 =plot(x(1),x(2),'ro','markersize',10,'markerfacecolor','r')
              sing = [sing; x(1),x(2)];   
        end
    end


    
end




if plotHeat
    xs = -100:10:100;
    ys = xs;
    
    for i=1:length(xs)
        for j = 1:length(ys)
            X = [xs(i),ys(j)];
            F = VF(X,vf);
            US(i,j) = F(1);
            VS(i,j) = F(2);
            XS(i,j) = xs(i);
            YS(i,j) = ys(j);
            
            mag(i,j) = sqrt(US(i,j)^2+VS(i,j)^2);
            
            US(i,j) = US(i,j)/mag(i,j);
            VS(i,j) = VS(i,j)/mag(i,j);
        end
    end    
    figure('pos',[10 10 900 500]);
    surf(XS,YS,mag)
    
    xlabel('x');
    ylabel('y');
    
    h = colorbar;
    ylabel(h, 'Vector Magnitude')
    shading interp
    view([0,90])
    set(gca,'fontsize',12);
    axis equal
    axis([-50,50,-50,50]);
end

end

function F = VF(X,vf)
%Compute values of each vector component
x = X(1);
y = X(2);

[U,V] = vf.heading(x,y);

F(1) = U;
F(2) = V;

% mag = sqrt(F(1)^2+F(2)^2);
% F(1) = F(1)/mag;
% F(2) = F(2)/mag;
end