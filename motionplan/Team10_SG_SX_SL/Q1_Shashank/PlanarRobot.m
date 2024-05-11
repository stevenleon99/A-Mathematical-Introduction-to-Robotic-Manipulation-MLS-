% Ref: https://www.mathworks.com/matlabcentral/fileexchange/56554-n-r-planar-robot?s_tid=FX_rc2_behav&status=SUCCESS

classdef PlanarRobot
    properties
        n;         % Number of links
        lengths;   % Lengths of each link
        theta;     % Current joint angles
        pts;       % Points of each joint in the workspace
        ee;        % End-effector position
        base;      % Base or mounting point of the robot [x, y]
    end
    %
    methods
        function obj = PlanarRobot(n, lengths, base)
            % Constructor to initialize the robot
            obj.n = n;
            obj.lengths = lengths;
            obj.theta = zeros(1, n);
            obj.base = base;  % Initialize the base position
            obj = obj.calculateAllPositions();
        end
        %
        function obj = calculateAllPositions(obj)
            % Calculate positions of all joints and update the end-effector
            obj.pts = [obj.base; obj.calculatePositions() + obj.base];  % Add the base to all joint positions
            obj.ee = obj.pts(end, :);  % Update end-effector position
        end
        %
        function pts = calculatePositions(obj)
            % Calculate positions of each joint
            d = zeros(1, obj.n);
            a = obj.lengths;
            alpha = zeros(1, obj.n);
            offset = zeros(1, obj.n);
            T = eye(4);
            pts = zeros(obj.n, 2);
            %
            for i = 1:obj.n
                theta_i = obj.theta(i) + offset(i);
                ct = cos(theta_i);
                st = sin(theta_i);
                ca = cos(alpha(i));
                sa = sin(alpha(i));
                %
                T = T * [
                    ct, -st*ca,  st*sa, a(i)*ct; ...
                    st,  ct*ca, -ct*sa, a(i)*st; ...
                    0,     sa,     ca,    d(i); ...
                    0,      0,      0,       1
                    ];
                pts(i, :) = T(1:2, 4)';  % Store the x and y position
            end
        end
        %
        function obj = updateJointAngles(obj, newTheta)
            % Update joint angles and recompute the positions
            if length(newTheta) ~= obj.n
                error('Number of joint angles must match the number of links.');
            end
            obj.theta = newTheta;
            obj = obj.calculateAllPositions();
        end
        %
        function collision = checkLinkObstacleCollision(obj, obstacles)
            % Check if any link intersects any obstacle
            collision = false;
            numLinks = size(obj.pts, 1) - 1; % minus base point
            %
            for i = 1:numLinks
                startPt = obj.pts(i, :);
                endPt = obj.pts(i+1, :);
                numInterpPts = 10;
                % Interpolate points between startPt and endPt
                interpPts = startPt;
                step = (endPt - startPt) / numInterpPts;
                for j = 1:numInterpPts
                    interpPts = [interpPts; interpPts(end, :) + step];
                end
                %
                % Check each point against each obstacle
                for j = 1:size(interpPts, 1)
                    for k = 1:length(obstacles)
                        if inpolygon(interpPts(j, 1), interpPts(j, 2), obstacles{k}(1, :), obstacles{k}(2, :))
                            collision = true;
                            return;
                        end
                    end
                end
            end
        end
    end
end
