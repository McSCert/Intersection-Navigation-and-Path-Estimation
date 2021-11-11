function [min_error, rel_error_percent, max_error] = test_map_roi(map, file_name)
    fid = fopen(file_name);
    tline = fgetl(fid);
    tlines = cell(0,1);
    while ischar(tline)
        tlines{end+1,1} = tline;
        tline = fgetl(fid);
    end
    fclose(fid);
    min_error = inf;
    max_error = -inf;
    avg_num =0;
    avg_dem =0;
    
    node = 0;
    for i = 1:1:length(tlines)
        
        if contains(tlines{i}, 'Right Rd Type: Within')
            intrsct_roi = map.get_roi(node);
            test_case_rl_line = tlines{i};
            test_case_td_line = tlines{i+1};
            rghtlft = intrsct_roi.get_rl_rgn().disp_rgn();
            tpdwn = intrsct_roi.get_td_rgn().disp_rgn();
            
            [min_error, max_error, avg_num, avg_dem] = compare_roi_strs(test_case_rl_line, rghtlft, min_error, max_error, avg_num, avg_dem);
            [min_error, max_error, avg_num, avg_dem] = compare_roi_strs(test_case_td_line, tpdwn, min_error, max_error, avg_num, avg_dem);
        elseif contains(tlines{i}, 'Top Rd Type: Within')
            continue
        elseif contains(tlines{i}, 'Node:') && contains(tlines{i}, 'Type:')
            node = node+1;
            intrsct_roi = map.get_roi(node);
            test_case_line = tlines{i};
            rgn = intrsct_roi.disp_rgn();
            
            [min_error, max_error, avg_num, avg_dem] = compare_roi_strs(test_case_line, rgn, min_error, max_error, avg_num, avg_dem);
            
        elseif contains(tlines{i}, 'Node:')
            node = node+1;
        end
        
    end
    
    rel_error_percent = 100*avg_num/avg_dem;
    
end



function [min_error, max_error, avg_num, avg_dem] = compare_roi_strs(test_case, rgn_values, min_error, max_error, avg_num, avg_dem)
    points = {'', 'x_srt_1 ', 'y_srt_1 ', 'x_srt_2 ', 'y_srt_2 ', 'x_mid_1 ', ...
        'y_mid_1 ', 'x_mid_2 ', 'y_mid_2 ', 'x_end_1 ', 'y_end_1 ', ...
        'x_end_2 ', 'y_end_2 '};
    test_case_split = split( test_case, ',' );
    rgn_values_split = split( rgn_values, ',' );
    if contains(test_case_split{1}, 'Node')
        test_case_split(1) = [];
    end
    
    for i = 2:1:length(test_case_split)
        tc_value  = str2num(strrep(test_case_split{i}, points{i}, ''));
        rgn_value = str2num(strrep(rgn_values_split{i}, points{i}, ''));
        
        abs_error = abs(tc_value - rgn_value);
        
        min_error = min(abs_error, min_error);
        max_error = max(abs_error, max_error);
        avg_num = avg_num + abs_error;
        avg_dem = avg_dem + (tc_value + rgn_value);
        
    end
end