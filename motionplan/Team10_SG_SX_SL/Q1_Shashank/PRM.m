classdef PRM
    % Class for Probabilistic Roadmap (PRM)
    %
    properties
        qI          % Initial Point [qI_1, qI_2, ... qI_dim]
        qG          % Goal Point [qG_1, qG_2, ... qG_dim]
        dim         % Dimension of the configuration space
        n           % Number of vertices to sample
        K           % Number of closest neighbors to consider
        O           % Cell array of obstacles, each represented as a 2xN matrix [x; y]
        qmin        % Minimum q_i value for each dimension
        qmax        % Maximum q_i value for each dimension
        robot       % Robot object
        V           % Vertices of the graph
        G_matrix    % Weighted adjacency matrix representing the graph
        G           % Graph object
    end
    %
    methods
        function obj = PRM(qI, qG, n, K, O, qmin, qmax, robot)
            % Constructor for PRM class
            % Inputs:
            %  qI: Initial Point [qI_1, qI_2, ... qI_dim]
            %  qG: Goal Point [qG_1, qG_2, ... qG_dim]
            %  n: Number of vertices to sample
            %  K: Number of closest neighbors to consider
            %  O: Cell array of obstacles, each represented as a 2xN matrix [x; y]
            %  qmin: Minimum q_i value for each dimension
            %  qmax: Maximum q_i value for each dimension
            %  robot: Robot object
            % Outputs:
            %  obj: PRM object
            obj.qI = qI;
            obj.qG = qG;
            obj.dim = length(qI);
            obj.n = n;
            obj.K = K;
            obj.O = O;
            obj.qmin = qmin;
            obj.qmax = qmax;
            obj.robot = robot;
            obj = obj.buildPRM();
        end
        %
        function obj = buildPRM(obj)
            % Build the PRM graph
            % Outputs:
            % obj: Updated PRM object
            % Initialize the graph with the initial point
            V = obj.qI;
            % Start count including the initial point
            count = 1;
            % Sample vertices
            while count < obj.n + 1
                % Sample a random point in the configuration space
                qrand = obj.qmin + rand(1, obj.dim) .* (obj.qmax - obj.qmin);
                % Check if the point is in collision
                robot_new = obj.robot.updateJointAngles(qrand);
                if ~robot_new.checkLinkObstacleCollision(obj.O)
                    % Add the point to the graph and increment the count
                    V = [V; qrand];
                    count = count + 1;
                end
            end
            % Append goal point
            obj.V = [V; obj.qG];
            %
            % Convert the vertices to cartesian space
            cart_V = zeros(size(obj.V, 1), 2);
            for i = 1:size(obj.V, 1)
                % Update the robot with the joint angles
                obj.robot = obj.robot.updateJointAngles(obj.V(i, :));
                cart_V(i, :) = obj.robot.ee;
            end
            %
            % Initialize the adjacency matrix
            obj.G_matrix = sparse(size(cart_V, 1), size(cart_V, 1));
            % Compute the edges
            for i = 1:size(cart_V, 1)
                % Compute the distance to all other points
                dists = sqrt(sum((cart_V - cart_V(i, :)).^2, 2));
                [~, idx] = sort(dists);
                % K closest neighbors, excluding the point itself
                neighbors = idx(2:min(end, obj.K+1));
                for j = neighbors'
                    % Check if the edge is in collision
                    if ~obj.isInCollision([cart_V(i, :); cart_V(j, :)])
                        % Graph is undirected
                        obj.G_matrix(i, j) = dists(j);
                        obj.G_matrix(j, i) = dists(j);
                    end
                end
            end
            % Create a graph object
            obj.G = graph(obj.G_matrix);
        end
        %
        function path = findPath(obj)
            % Find the path from the initial point to the goal point
            % Outputs:
            % path: Path from the initial point to the goal point
            % Find the indices of the start and goal points
            startIdx = find(ismember(obj.V, obj.qI, 'rows'));
            goalIdx = find(ismember(obj.V, obj.qG, 'rows'));
            % Find the shortest path
            [idx_path, ~] = shortestpath(obj.G, startIdx, goalIdx);
            % Extract the path from the graph
            path = obj.V(idx_path, :);
        end
        %
        function inCollision = isInCollision(obj, S)
            % Check if the edge is in collision with the obstacles
            % Inputs:
            %  S: 2x2 matrix representing the edge [x1, y1; x2, y2]
            % Outputs:
            %  inCollision: Flag indicating if the edge is in collision
            inCollision = false;
            % Check for collision with each obstacle
            for i = 1:length(obj.O)
                % Check if the edge intersects with the polygon
                if obj.isintersect_linepolygon(S, obj.O{i})
                    inCollision = true;
                    return;
                end
            end
        end
    end
    methods (Access = private)
        function b = isintersect_linepolygon(obj, S, Q)
            % Check if the line segment S intersects with the polygon Q
            % Inputs:
            %  S: Line segment represented as a 2x2 matrix [x1, x2; y1, y2]
            %  Q: Polygon represented as a 2xN matrix [x; y]
            % Outputs:
            %  b: Flag indicating if the line segment intersects with the polygon
            % % Extract the start and end points of the line segment
            p0 = S(1,:);
            p1 = S(2,:);
            %
            % Check if the line segment is a point
            if p0 == p1
                b = inpolygon(p0(1), p0(2), Q(1,:), Q(2,:));
                return;
            end
            %
            % Check if the line segment intersects with any of the polygon edges
            n = size(Q, 2);
            b = false;
            %
            for i = 1:n
                % Extract the edge vertices
                q0 = Q(:, i)';
                if i == n
                    q1 = Q(:, 1)';
                else
                    q1 = Q(:, i+1)';
                end
                %
                % Check if the line segment intersects with the edge
                if obj.doIntersect(p0, p1, q0, q1)
                    b = true;
                    return;
                end
            end
        end
        %
        function flag = doIntersect(obj, p0, p1, q0, q1)
            % Check if the line segments p0p1 and q0q1 intersect
            % Inputs:
            %  p0, p1: Start and end points of the first line segment
            %  q0, q1: Start and end points of the second line segment
            % Outputs:
            %  flag: Flag indicating if the line segments intersect
            % Compute the direction vectors
            ds = p1 - p0;
            n1 = q1 - q0;
            n2 = p0 - q0;
            % Compute the determinant
            ds_n_dot = ds(1) * n1(2) - ds(2) * n1(1);
            %
            % Check if the line segments are parallel
            if ds_n_dot == 0
                % Check if the line segments are collinear
                tL = dot(n2, n1) / dot(n1, n1);
                tE = dot(n2 + ds, n1) / dot(n1, n1);
                flag = (min(tL, tE) <= 1) && (max(tL, tE) >= 0);
            else
                % Compute the parameters for the intersection point
                tL = (n2(2) * n1(1) - n2(1) * n1(2)) / ds_n_dot;
                tE = (n2(2) * ds(1) - n2(1) * ds(2)) / ds_n_dot;
                flag = (tL >= 0) && (tL <= 1) && (tE >= 0) && (tE <= 1);
            end
        end
    end
end