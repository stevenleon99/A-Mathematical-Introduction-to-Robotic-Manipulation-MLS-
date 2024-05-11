function theta = inverseKinematics(robot, target, maxIter, tol)
% Inputs:
% robot - Instance of PlanarRobot
% target - Desired [x, y] position of the end-effector
% maxIter - Maximum number of iterations for the algorithm
% tol - Tolerance for error in reaching the target
% Outputs:
% theta - Joint angles for the desired end-effector position

% Current end-effector position
currentPos = robot.ee;
% Current joint angles
theta = robot.theta;

% Iterate until convergence
for iter = 1:maxIter
    %
    % Check if the end-effector is close to the target
    if norm(currentPos - target) < tol
        break;
    end
    %
    % Calculate the Jacobian matrix
    J = calculateJacobian(robot, theta);
    %
    % Error between current end-effector position and target
    error = target - currentPos;
    %
    % Update joint angles using Jacobian transpose
    step_size = 0.1;  % Step size for update
    dTheta = step_size * pinv(J) * error';
    %
    % Update joint angles
    theta = theta + dTheta';
    %
    % Update robot configuration
    robot = robot.updateJointAngles(theta);
    currentPos = robot.ee;
end

% Check if the algorithm converged
if iter == maxIter
    disp('Max iterations reached without convergence.');
    theta = [];
end
end

function J = calculateJacobian(robot, theta)
% Calculate the Jacobian matrix for the given joint angles
% Inputs:
% robot - Instance of PlanarRobot
% theta - Joint angles
% Outputs:
% J - Jacobian matrix
% Small number for numerical derivative
eps = 1e-6;
% Initialize Jacobian matrix
J = zeros(2, robot.n);

% Calculate initial positions
initialPositions = robot.calculatePositions();

% Calculate Jacobian matrix using numerical differentiation
for i = 1:robot.n
    % Perturb the joint angle
    thetaPlus = theta;
    thetaPlus(i) = thetaPlus(i) + eps;
    %
    % Calculate new positions with perturbed angles
    robot.theta = thetaPlus;
    posPlus = robot.calculatePositions();
    %
    % Derivative (difference quotient)
    J(:, i) = (posPlus(end, :) - initialPositions(end, :))' / eps;
end
end
