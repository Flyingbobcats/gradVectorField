


function p = activationFunctions(theta)

    if theta<deg2rad(90)
        p = 1/2*tanh(20*theta-pi)+1;
    else
        p = -1/2*tanh(5*theta-5*pi)+1;
    end


end



% 
% clc
% clear
% close all
% 
% thetas = 0:0.01:2*pi;
% 
% 
% 
% 
% 
% for i=1:length(thetas)
% 
%     if thetas(i)<deg2rad(90)
%         p(i) = 1/2*tanh(20*thetas(i)-pi)+1;
%     else
%         p(i) = -1/2*tanh(5*thetas(i)-5*pi)+1;
%     end
% 
% end
% 
% plot(rad2deg(thetas),p)