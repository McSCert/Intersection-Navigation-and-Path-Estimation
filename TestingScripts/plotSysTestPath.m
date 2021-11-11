function plotSysTestPath(scenario, i)
    plot(scenario);
    hold on;
    [v1_est,v1_act] = plot_paths(['test_' num2str(i) '_vehicle_1_estimated_pos.mat'],['test_' num2str(i) '_vehicle_1_feedback_pos.mat']);
    [v2_est,v2_act] = plot_paths(['test_' num2str(i) '_vehicle_2_estimated_pos.mat'],['test_' num2str(i) '_vehicle_2_feedback_pos.mat']);
    
    legend([v1_est,v1_act, v2_est,v2_act], {'Vehicle 1 estimated path',  'Vehicle 1 actual path' ...
        'Vehicle 2 estimated path',  'Vehicle 2 actual path'});
end