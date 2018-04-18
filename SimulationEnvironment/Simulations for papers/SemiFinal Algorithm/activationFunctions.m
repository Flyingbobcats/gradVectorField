


function p = activationFunctions(theta,type)



if type =='r'
    if theta<deg2rad(90)
        p = 1/2*tanh(5*theta-pi/2)+0.5;
    else
        p = -1/2*tanh(5*theta-6*pi)+0.5;
    end

elseif type =='a'
%         if theta<deg2rad(90)
%         p = -1/2*tanh(30*theta-pi)+1;
%     else
%         p = 1/2*tanh(10*theta-10*pi)+1;
%         end
        
        p=1;


        
elseif type == 'g'   
        p = -1/2*tanh(2.5*theta-pi)+0.5;   
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



% clc
% clear
% close all
% 
% thetas = 0:0.01:2*pi;
% 
% for i=1:length(thetas)
%         theta = thetas(i);
%  p(i) = -1/2*tanh(3.5*theta-pi)+0.5;
% end
% 
% plot(rad2deg(thetas),p)



