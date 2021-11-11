%% Plot the paths of motion for two vehicles
% [P1,P2]= PLOT_PATHS(FILE1,FILE2) Plots the path of vehicle 1 from
% FILE1 and vehicle 2 from FILE2. 
%
% OUTPUTS
%
% P1 - the plot handle of the first path
% P2 - the plot handle of the second path
function [p1,p2]= plot_paths(file1,file2)
    m = matfile(file1,'Writable',true);
    path1 = m.path;
    path1 =path1';
    m = matfile(file2,'Writable',true);
    path2 = m.path;
    path2 =path2';
    
    hold on;
    p1 = plot(path1(:,1),path1(:,2),'LineWidth',4);
    p2 =plot(path2(:,1),path2(:,2),'LineWidth',4);
    
end

