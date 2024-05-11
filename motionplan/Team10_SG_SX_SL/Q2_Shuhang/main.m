clear; clc; close all;

videoFile = 'task2.mp4'; % Name of the video file
videoWriter = VideoWriter(videoFile, 'MPEG-4');
videoWriter.FrameRate = 10; % Set the frame rate
open(videoWriter);

%% Obstacles
O1 = [1, 3, 3, 1; 1, 1, 3, 3];  % Square
O2 = [4, 6, 6; 1, 1, 3];        % Right-angled triangle
O3 = [7, 9, 9, 7; 4, 4, 6, 6];  % Rectangle
O4 = [1, 3, 5, 4; 5, 7, 7, 5];  % Irregular quadrilateral
O5 = [8, 10, 10, 8; 1, 1, 2, 2]; % Small rectangle
O6 = [5, 7, 6; 8, 8, 10];        % Isosceles triangle
obs_set = {O1, O2, O3, O4, O5, O6};

%% Initial point and goal point pos (center point)
qI = [9;0.5;0];
qG = [2;8;pi/4];

%% Boundary of the field
bounds=[10 10 0 0;0 10 10 0];

%% Robot boundary
length = 1;
width = 0.3;
robot = [length/2, -width/2;
         length/2, width/2;
         -length/2, width/2;
         -length/2, -width/2;
         length/2, -width/2]';
robot = [robot; 1,1,1,1,1];

%% Plot the obs and boundary
hold on;
title("Robot Motion Planning Task 2");
plot([bounds(1,:),bounds(1,1)],[bounds(2,:),bounds(2,1)]);
axis equal;
for i = 1:size(obs_set,2)
    obs = obs_set{i};
    Obs = polyshape([obs(1,:), obs(1,1)], [obs(2,:), obs(2,1)]);
    plot(Obs);
end

%% Call the APF to find the path
[path] = APF(qI, qG, obs_set, length);

%% Only plot the start point, end point and one point in every 5 point
for i = 1:size(path,2)
    robotCenterPoint = path(:,i);
    theta = robotCenterPoint(3);
    x = robotCenterPoint(1);
    y = robotCenterPoint(2);
    R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    robot_trans = [R, [x;y]; 0, 0, 1] * robot;
    if i == 1
        p1 = plot(robot_trans(1,:), robot_trans(2,:), 'r', 'LineWidth', 2, 'DisplayName', 'Start Point');
    elseif i == size(path,2)
        p2 = plot(robot_trans(1,:), robot_trans(2,:), 'g', 'LineWidth', 2, 'DisplayName', 'End Point');
    elseif mod(i,5) == 1
        plot(robot_trans(1,:), robot_trans(2,:), 'k', 'HandleVisibility', 'off');
        drawnow; % Update figure window
        frame = getframe(gcf); % Capture frame
        writeVideo(videoWriter, frame); % Write frame to video
        pause(0.1);
    end
end
% Show the legend of start point and end point
legend([p1, p2]);

% Add a pause at the end of the video
numPauseFrames = 3 * videoWriter.FrameRate; % 3 seconds * frame rate
lastFrame = getframe(gcf); % Capture the last frame
for k = 1:numPauseFrames
    writeVideo(videoWriter, lastFrame); % Write the last frame repeatedly
end
close(videoWriter);
