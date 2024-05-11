% Clear the workspace
clear; clc; close all;

% Define the obstacles
B = {
    [30, 60; 15, 65; 20, 40; 30, 35; 40, 50]';
    [50 20; 75 15; 70 40; 50 40]';
    [50 80; 80 60; 85 70]';
    };
% Define the colors for the obstacles
obs_color = ['r', 'g', 'b'];

% Define the initial and goal positions
p_init = [15, 75];
p_goal = [80, 35];

% Define the world limits
L_world = 100;

% Define the robot
robot = PlanarRobot(5, [15, 15, 10, 5, 5], [50, 60]);

% Select the planning method
choice = input('Select Planning Method -\n1. PRM\n2. RRT\nEnter Choice: ');
q_init = mod(inverseKinematics(robot, p_init, 10000, 0.01), 2 * pi);
q_goal = mod(inverseKinematics(robot, p_goal, 10000, 0.01), 2 * pi);
if choice == 1
    % PRM
    N = 1000;
    K = 15;
    prm = PRM(q_init, q_goal, N, K, B, 0, 2*pi, robot);
    Path = prm.findPath();
else
    % RRT
    N = 5000;
    delta_q = 1;
    tolerance = 3;
    rrt = RRT(q_init, q_goal, N, delta_q, B, 0, 2*pi, robot, tolerance);
    Path = rrt.findPath();
end

% Setup the figure
figure;
hold on;
axis([0 L_world 0 L_world]);

% Plot obstacles
for i = 1:length(B)
    plot(polyshape(B{i}(1,:), B{i}(2,:)), 'FaceColor', obs_color(i), 'EdgeColor', obs_color(i));
end

% Plot initial and goal positions
plot(p_init(1), p_init(2), 'bo', 'MarkerSize', 5);
plot(p_goal(1), p_goal(2), 'ro', 'MarkerSize', 5);

% Initialize plot for robot path
h_path = plot(0, 0, '-bo', 'MarkerFaceColor', 'b', 'LineWidth', 2, 'MarkerSize', 6);
h_robot = plot(0, 0, 'o', 'MarkerSize', 6, 'MarkerFaceColor', 'y', 'MarkerEdgeColor', 'k');

% Animate the robot
disp('Click on the Graph to Start the Animation');
waitforbuttonpress;

for ix = 1:size(Path, 1) - 1
    p = robot.updateJointAngles(Path(ix, :)).ee;
    p_next = robot.updateJointAngles(Path(ix + 1, :)).ee;
    for step = 0:0.1:1
        p_step = p + step * (p_next - p);
        q_step = mod(inverseKinematics(robot, p_step, 10000, 0.01), 2 * pi);
        robot = robot.updateJointAngles(q_step);
        if robot.checkLinkObstacleCollision(B)
            continue;
        end
        set(h_robot, 'XData', robot.ee(1), 'YData', robot.ee(2));
        set(h_path, 'XData', robot.pts(:, 1), 'YData', robot.pts(:, 2));
        drawnow;
        pause(0.1);
    end
end

hold off;
disp('Animation Completed');
disp('Click on the Graph to Close the Figure');
waitforbuttonpress;
close all;