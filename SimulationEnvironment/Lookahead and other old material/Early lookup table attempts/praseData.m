
close all


[numRows,numColumns] = size(XSOLVED);

for i = 1:numRows
    for j = 1:numColumns
    hs(i,j) = XSOLVED{i,j}(1); 
    ks(i,j) = XSOLVED{i,j}(2);
    vs(i,j) = VS(i,j)/0.35;
    ns(i,j) = NS(i,j);
    costs(i,j) = COSTS(i,j);
    
    end
end

% figure
% surface(vs,ns,ks);
% xlabel('turn radius');
% ylabel('n');
% zlabel('ks');
% 
% figure
% surface(vs,ns,hs)
% xlabel('turn radius');
% ylabel('n');
% zlabel('hs');

figure
surface(VS,NS,costs)
xlabel('turn radius');
ylabel('n');
zlabel('costs');


% for i = 1:numRows
%     for j = 1:numColumns
%     hs(i,j) = XSOLVED{i,j}(1); 
%     ks(i,j) = XSOLVED{i,j}(2);
%     costs(i,j) = COSTS(i,j);
%     
%     end
% end
% 
% figure
% surface(ks,hs,costs);
% xlabel('ks');
% ylabel('hs');
% zlabel('costs');



