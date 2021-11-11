%% Add to path of motion file
% APPEND_TO_PATH(FILE_NAME,NEWDATA) adds the data (NEWDATA) into the 
% file (FILE_NAME)
function append_to_path(file_name,newdata)
    % mydata.mat only has a 'variable' - 
    % it is a 2 rows, 1 column cell by cell named mydatacell 
    % which is row 1 data, and row 2 the label name
    %Define Matfile
    m = matfile(file_name,'Writable',true);
    
    %Append the data
    if ismember('path', fieldnames(m))         %is the variable already present?
        [nrows, ncols] = size(m, 'path');
        path =[m.path  newdata];             %yes, extend it
    else
        path = newdata;                      %no, assign it
    end
    save(file_name,'path');
end

