points = [1000 2000 3000 4000 5000 6000 7000 8000 9000 10000 20000];
average_error = zeros(1,length(points));
max_error = zeros(1,length(points));
for i =1:1:length(points)
    sprintf('Running For %e', points(i) )
    map = Map( 'MarcCity.mat', 1/(points(i)));
    [min_error, average_error(i), max_error(i)] = test_map_roi(map, 'marc_city_roi.txt');
end

plot(points, average_error,'LineWidth', 2);
hold on
plot(points, max_error,'LineWidth', 2);
title('Number of Points Vs. Error')
xlabel('points (#)')
ylabel('Error')
legend('SMAPE (%)', 'Max Error (m)')