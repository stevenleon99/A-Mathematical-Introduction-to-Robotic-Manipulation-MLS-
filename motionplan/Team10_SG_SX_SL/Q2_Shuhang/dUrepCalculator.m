function dUrep = dUrepCalculator(q, Q_star, eta, obstacleBoundaries)
    % Initialize the repulsive gradient vector
    dUrep = [0; 0];

    % Iterate over each obstacle
    for i = 1:size(obstacleBoundaries, 3)
        % Extract the i-th obstacle line segment
        Obs_i = obstacleBoundaries(:, :, i);

        % Initialize minimum dist as very large to ensure it gets updated
        minDist = inf;
        cp = [0; 0];

        % Iterate through points on the obstacle to find the closest point
        for j = 1:size(Obs_i, 2)
            [closest_pt, dist] = closestPointOnObsToQ(Obs_i, q);

            % Update the closest point and minimum dist if a closer point is found
            if dist < minDist
                cp = closest_pt;
                minDist = dist;
            end
        end

        % Compute the repulsive gradient if the minimum dist is within the effective range
        if minDist <= Q_star
            d_rho_q = (q - cp) / minDist;
            dUrep = dUrep + eta * (1/Q_star - 1/minDist) * (1/minDist^2) * d_rho_q;
        else
            dUrep = dUrep;
        end
    end
end
