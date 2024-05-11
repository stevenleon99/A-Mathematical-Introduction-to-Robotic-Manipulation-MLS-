function [path, num_steps] = a_star(grid, start_pos, goal_pos, heuristic_type)
%  Perform A* search on a grid with a given heuristic with the specified
%  start_pos and goal_pos. the heuristic_type which is a string that
%  can be 'manhattan', 'euclidean', or 'dijkstra'.
% Inputs:
%   grid: A 2D character array representing the grid.
%   start_pos: A 1x2 vector representing the starting position [row, col].
%   goal_pos: A 1x2 vector representing the goal position [row, col].
%   heuristic_type: A string specifying the heuristic type ('manhattan', 'euclidean', 'dijkstra').
% Outputs:
%   path: A Nx2 matrix representing the path from start_pos to goal_pos.
%   num_steps: An integer representing the number of steps taken to reach the goal.

% Define the heuristic function
switch heuristic_type
    case 'manhattan'
        % Manhattan distance heuristic
        heuristic = @(pos) abs(pos(1) - goal_pos(1)) + abs(pos(2) - goal_pos(2));
    case 'euclidean'
        % Euclidean distance heuristic
        heuristic = @(pos) sqrt((pos(1) - goal_pos(1))^2 + (pos(2) - goal_pos(2))^2);
    case 'dijkstra'
        % Dijkstra is essentially A* with no heuristic
        heuristic = @(pos) 0;
end

% Initialize the open set (frontier)
openSet = containers.Map('KeyType', 'char', 'ValueType', 'any');
% Initialize the cameFrom map
cameFrom = containers.Map('KeyType', 'char', 'ValueType', 'any');

% Start position key and initialization
startKey = pos2key(start_pos);
goalKey = pos2key(goal_pos);
% Initialize the open set with the start position
% Format: openSet(key) = [gScore, hScore, fScore]
openSet(startKey) = [0, heuristic(start_pos), 0 + heuristic(start_pos)];

% Priority queue contains [fScore, row, col]
priorityQueue = [heuristic(start_pos), start_pos];

% A* search
while ~isempty(priorityQueue)
    % Sort the priority queue by fScore
    [~, idx] = min(priorityQueue(:, 1));
    current = priorityQueue(idx, 2:3);
    currentKey = pos2key(current);
    % Remove the current node from queue
    priorityQueue(idx, :) = [];
    %
    % Check if we have reached the goal
    if strcmp(currentKey, goalKey)
        path = reconstruct_path(cameFrom, goalKey);
        num_steps = size(path, 1) - 1;
        return;
    end
    %
    % Extract current costs from openSet
    currentCosts = openSet(currentKey);
    currentGScore = currentCosts(1);
    %
    % Explore neighbors (4-directional)
    % transpose to loop over columns
    for d = [-1, 0; 1, 0; 0, -1; 0, 1]'
        neighbor = current + d';
        neighborKey = pos2key(neighbor);
        if is_within_grid(grid, neighbor) && grid(neighbor(1), neighbor(2)) ~= '⬛'
            % Increment the gScore
            tentative_gScore = currentGScore + 1;
            %
            % Check if neighborKey is not in openSet or has a higher gScore than current tentative_gScore
            if ~isKey(openSet, neighborKey)
                isNewKey = true;
            else
                neighborCosts = openSet(neighborKey);
                isNewKey = tentative_gScore < neighborCosts(1);
            end
            %
            if isNewKey
                cameFrom(neighborKey) = currentKey;
                fScore = tentative_gScore + heuristic(neighbor);
                openSet(neighborKey) = [tentative_gScore, heuristic(neighbor), fScore];
                % Add to priority queue
                priorityQueue(end+1, :) = [fScore, neighbor];
            end
        end
    end
end

% If no path is found
path = [];
num_steps = 0;
end

function key = pos2key(pos)
% Convert position to a unique key
% Format: 'row-col'
% Example: '3-4'
key = sprintf('%d-%d', pos(1), pos(2));
end

function pos = key2pos(key)
% Convert key to position
% Example: '3-4' -> [3, 4]
indices = find(key == '-', 1);
x = str2double(key(1:indices-1));
y = str2double(key(indices+1:end));
pos = [x, y];
end

function path = reconstruct_path(cameFrom, currentKey)
% Reconstruct the path from the cameFrom map
% Inputs:
%   cameFrom: A map containing the parent node for each node.
%   currentKey: The key of the current node.
% Outputs:
%   path: A Nx2 matrix representing the path from start to goal.
path = [];
while isKey(cameFrom, currentKey)
    currentPos = key2pos(currentKey);
    path = [path; currentPos];
    currentKey = cameFrom(currentKey);
end
% Reverse the path to start to goal
path = flipud(path);
end

function inside = is_within_grid(grid, pos)
% Check if the position is within the grid and not an obstacle
inside = all(pos > 0) && pos(1) <= size(grid, 1) && pos(2) <= size(grid, 2) && grid(pos(1), pos(2)) ~= '⬛';
end
