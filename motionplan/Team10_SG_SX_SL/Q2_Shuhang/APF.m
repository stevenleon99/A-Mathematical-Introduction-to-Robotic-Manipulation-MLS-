function [path] = APF(qI, qG, O, length)
    % Initialize constants
    Q_goal = 5;
    Q_obs = 1;
    numOfCP = 4;
    zeta = 0.1;
    eta = 0.1;
    tolerance = 0.001;
    rotationalFactor = 6;
    alpha = 0.1;
    % Find the maximum number of vertices in any obstacle
    maxVertices = max(cellfun(@(x) size(x, 2), O));

    % Prepare obstacle data structure
    obstacleBoundaries = NaN(2, maxVertices + 1, numel(O));  % Use NaN to initialize
    for i = 1:numel(O)
        numVertices = size(O{i}, 2);
        obstacleBoundaries(:, 1:numVertices, i) = O{i};
        obstacleBoundaries(:, numVertices + 1, i) = O{i}(:, 1);  % Closing the loop
    end

    % Generate control points using vectorization
    indices = 0:(numOfCP-1);  % Array of indices from 0 to numOfCP-1
    positions = (-0.5 + indices / (numOfCP - 1)) * length;  % Compute x-coordinates
    cp = [positions; zeros(1, numOfCP)];  % Stack x-coordinates and y-coordinates

    % Start gradient descent from initial configuration
    path = qI;
    q_curr = path(:, end);
    while true
        % Initialize u
        u = zeros(3, 1);
        
        % Calculate gradient contributions from each control point
        for i = 1:numOfCP
            R = [cos(q_curr(3)), -sin(q_curr(3)); sin(q_curr(3)), cos(q_curr(3))];
            pt = q_curr(1:2) + R * cp(:, i);
            J = [eye(2), [-cp(1, i) * sin(q_curr(3)); cp(1, i) * cos(q_curr(3))]];
            dUatt = dUattCalculator(pt, qG, cp(:, i), Q_goal, zeta);
            dUrep = dUrepCalculator(pt, Q_obs, eta, obstacleBoundaries);
            u = u + J' * (dUatt + dUrep);
        end
        
        % Update configuration
        updateMatrix = diag([ones(1, 2), rotationalFactor]);
        q_next = q_curr - alpha * updateMatrix * u;
        path = [path, q_next];
        q_curr = q_next;
        
        % Check for convergence
        % If they almost fit, terminate and return path
        if norm(q_next - qG) < tolerance
            break;
        end
    end
end
