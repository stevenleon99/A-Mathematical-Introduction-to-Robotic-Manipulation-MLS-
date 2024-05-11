clc; clear; close all;

% Read the grids from the text file
file_path = 'sample.txt';
grids = read_grids_from_file(file_path);

% Initialize performance metrics
manhattan_steps = zeros(1, length(grids));
euclidean_steps = zeros(1, length(grids));
dijkstra_steps = zeros(1, length(grids));
manhattan_times = zeros(1, length(grids));
euclidean_times = zeros(1, length(grids));
dijkstra_times = zeros(1, length(grids));
grid_sizes = zeros(1, length(grids));  % To store the sizes of each grid
manhattan_path_lengths = zeros(1, length(grids));
euclidean_path_lengths = zeros(1, length(grids));
dijkstra_path_lengths = zeros(1, length(grids));

% Process each grid
for i = 1:length(grids)
    disp("Processing Grid " + i + " of " + length(grids));
    grid = grids{i};
    start_row = size(grid, 1);  % Last row
    start_col = 1;              % First column
    goal_row = 1;               % First row
    goal_col = size(grid, 2);   % Last column
    grid_sizes(i) = numel(grid);  % Compute the size of the grid
    %
    % Time and perform A* with Manhattan heuristic
    tic;
    [path, manhattan_steps(i)] = a_star(grid, [start_row, start_col], [goal_row, goal_col], 'manhattan');
    manhattan_times(i) = toc;
    manhattan_path_lengths(i) = size(path, 1); % Get path length
    disp("      Completed Manhattan Heuristic");
    %
    % Time and perform A* with Euclidean heuristic
    tic;
    [path, euclidean_steps(i)] = a_star(grid, [start_row, start_col], [goal_row, goal_col], 'euclidean');
    euclidean_times(i) = toc;
    euclidean_path_lengths(i) = size(path, 1); % Get path length
    disp("      Completed Euclidean Heuristic");
    %
    % Time and perform A* with Dijkstra's algorithm (no heuristic)
    tic;
    [path, dijkstra_steps(i)] = a_star(grid, [start_row, start_col], [goal_row, goal_col], 'dijkstra');
    dijkstra_times(i) = toc;
    dijkstra_path_lengths(i) = size(path, 1); % Get path length
    disp("      Completed Dijkstra's Algorithm");
    %
    disp("Completed Grid " + i + " of " + length(grids));
    disp("--------------------------------------------------");
    disp(" ");
end

% Convert times from seconds to milliseconds for better readability
manhattan_times = manhattan_times * 1000;
euclidean_times = euclidean_times * 1000;
dijkstra_times = dijkstra_times * 1000;

% Calculate the scores
manhattan_score = (manhattan_times .* manhattan_steps .* manhattan_path_lengths) ./ grid_sizes;
euclidean_score = (euclidean_times .* euclidean_steps .* euclidean_path_lengths) ./ grid_sizes;
dijkstra_score = (dijkstra_times .* dijkstra_steps .* dijkstra_path_lengths) ./ grid_sizes;

% Create a single figure with three subplots
figure;

% Subplot 1: Performance comparison for steps
subplot(2, 2, 1);  % 2 rows, 2 columns, 1st subplot
plot(grid_sizes, manhattan_steps, '-o', 'DisplayName', 'Manhattan Steps');
hold on;
plot(grid_sizes, euclidean_steps, '-+', 'DisplayName', 'Euclidean Steps');
plot(grid_sizes, dijkstra_steps, '-x', 'DisplayName', 'Dijkstra Steps');
hold off;
legend('show', 'Location', 'northwest');
xlabel('Grid Size (number of cells)');
ylabel('Number of Steps to Target');
title('Steps Comparison');

% Subplot 2: Path length comparison
subplot(2, 2, 2);  % 2 rows, 2 columns, 2nd subplot
plot(grid_sizes, manhattan_path_lengths, '-o', 'DisplayName', 'Manhattan Path Length');
hold on;
plot(grid_sizes, euclidean_path_lengths, '-+', 'DisplayName', 'Euclidean Path Length');
plot(grid_sizes, dijkstra_path_lengths, '-x', 'DisplayName', 'Dijkstra Path Length');
hold off;
legend('show', 'Location', 'northwest');
xlabel('Grid Size (number of cells)');
ylabel('Path Length');
title('Path Length Comparison');

% Subplot 3: Time comparison
subplot(2, 2, 3);  % 2 rows, 2 columns, 3rd subplot
plot(grid_sizes, manhattan_times, '-o', 'DisplayName', 'Manhattan Time');
hold on;
plot(grid_sizes, euclidean_times, '-+', 'DisplayName', 'Euclidean Time');
plot(grid_sizes, dijkstra_times, '-x', 'DisplayName', 'Dijkstra Time');
hold off;
legend('show', 'Location', 'northwest');
xlabel('Grid Size (number of cells)');
ylabel('Computation Time (ms)');
title('Time Comparison');

% Subplot 4: Overall scores
subplot(2, 2, 4);  % 2 rows, 2 columns, 4th subplot
plot(grid_sizes, manhattan_score, '-o', 'DisplayName', 'Manhattan Score');
hold on;
plot(grid_sizes, euclidean_score, '-+', 'DisplayName', 'Euclidean Score');
plot(grid_sizes, dijkstra_score, '-x', 'DisplayName', 'Dijkstra Score');
hold off;
legend('show', 'Location', 'northwest');
xlabel('Grid Size (number of cells)');
ylabel('Score');
title('Overall Scores');

% Adjust layout to prevent overlap
sgtitle('Comparison of Pathfinding Heuristics');  % Super title for the entire figure
