TestCaseMaps = {'TestCase1', 'TestCase2', 'TestCase3', 'TestCase4', ...
    'TestCase5', 'TestCase6', 'TestCase7', 'TestCase8' ...
    'TestCase9', 'MarcCity', 'NumOfRoad8'};

TestCaseROIs = {'test_case_1_roi', 'test_case_2_roi', 'test_case_3_roi', 'test_case_4_roi', ...
    '', 'test_case_6_roi', 'test_case_7_roi', 'test_case_8_roi' ...
    'test_case_9_roi', 'marc_city_roi', 'num_of_roads_8'};
CheckPerc = [true, true, true, true, ...
    false, true, true, true, ...
    true, true, true];

currentFolder = pwd;
thou_results= zeros(length(TestCaseMaps), 3);
ten_thou_results= zeros(length(TestCaseMaps), 3);
for i = 1:1:length(TestCaseMaps)
    if CheckPerc(i)
        
        thou_map = Map([TestCaseMaps{i} '.mat'], 10e-4);
        ten_thou_map = Map([TestCaseMaps{i} '.mat'], 10e-5);
        [thou_results(i,1), thou_results(i,2), thou_results(i,3)] = ...
            test_map_roi(thou_map, [TestCaseROIs{i} '.txt']);
        
        [ten_thou_results(i,1), ten_thou_results(i,2), ten_thou_results(i,3)] = ...
            test_map_roi(ten_thou_map, [TestCaseROIs{i} '.txt']);
        
        map = ten_thou_map;
    else
        map = Map([TestCaseMaps{i} '.mat'], 10e-5);
    end
    
    map.disp_graph;
    set(gcf, 'Position',  [0, 0, 1080, 1080]);
    hold off;
    saveas(gcf,[currentFolder '/SavedObjs/' TestCaseMaps{i} '.png']); 
end