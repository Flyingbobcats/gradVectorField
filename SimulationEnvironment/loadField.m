%Common Vector Field Scenarios (Quick loading)


function vectorFieldObject = loadField(v,options)


    if strcmp(options,'linecirc')
        %Straight path with a circular repulsive field placed at the origin
        v = v.navf('line');
        v.avf{1}.angle = pi/2;
        v = v.nrvf('circ');
        v.rvf{1}.r = 0.1;
        v.rvf{1} = v.rvf{1}.modDecay('hyper');
        
     
    elseif strcmp(options,'circOps')
        v = v.nrvf('circ');
        v.rvf{1}.r = 0.1;
        v.rvf{1} = v.rvf{1}.modDecay('hyper');
        v.NormSummedFields = false;
        

        
    else
        str = strcat(options, ' is not a valid input argument');
        warning(str);
    end
    
    
    
    vectorFieldObject = v;
end