function grids = read_grids_from_file(file_path)
% Reads grids from a text file and returns them as a cell array of character arrays.
% Each character array represents a grid.
% The text file should contain grids separated by an empty line.
% Inputs:
%   file_path: Path to the text file containing grids.
% Outputs:
%   grids: Cell array of character arrays representing the grids.

% Opens the text file containing grids
fid = fopen(file_path, 'r');
% Check if the file can be opened
if fid == -1
    error('File cannot be opened: %s', file_path);
end

% Initialize storage for the grids
grids = {};
grid = [];

% Read the file line by line
while ~feof(fid)
    % Read a line from the file
    line = char(fgetl(fid));
    % Remove newline character
    line = line(1:end-1);
    if isempty(line)
        % New grid begins after an empty line
        if ~isempty(grid)
            % Store the grid in the cell array and  Convert cell array to character array
            grids{end + 1} = char(grid);
            grid = [];
        end
    else
        % Append new row to the grid and Store row in cell array to handle variable lengths
        grid = [grid; {line}];
    end
end

% Add the last grid if file does not end with a newline
if ~isempty(grid)
    % Convert cell array to character array
    grids{end + 1} = char(grid);
end

% Close the file
fclose(fid);
end
