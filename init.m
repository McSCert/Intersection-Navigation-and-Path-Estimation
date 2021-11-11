addpath(fullfile(matlabroot,'examples','mpc','main'));
currentFolder = pwd;
load([currentFolder '/Buses/busses.mat'])

file_lines = strsplit(fileread('test1.txt'), newline);


for i =1:1:length(file_lines)
    line = strtrim(file_lines{i});
    if isempty(line) || line(1) == '%'
        continue
    else
        split_var = strsplit(line,',');
        var_name = strtrim(split_var{1});
        var_value = eval(strtrim(split_var{2}));
        assignin('base', var_name, var_value);
    end
end

simulation_name = evalin('base', 'simulation_name');
[scenario,egoCar,actor_Profiles] = helperSessionToScenario(simulation_name);
clc;
