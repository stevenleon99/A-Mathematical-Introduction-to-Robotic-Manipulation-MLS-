classdef RRT
    % Class for Rapidly-exploring Random Trees (RRT)
    %
    properties
        qI              % Initial Point [qI_1, qI_2, ... qI_dim]
        qG              % Goal Point [qG_1, qG_2, ... qG_dim]
        dim             % Dimension of the configuration space
        n               % Number of vertices to sample
        del_q           % Step size for extending the tree
        O               % Cell array of obstacles, each represented as a 2xN matrix [x; y]
        qmin            % Minimum q_i value for each dimension
        qmax            % Maximum q_i value for each dimension
        robot           % Robot object
        tolerance       % Tolerance for reaching the goal (Default: 3.0)
        V               % Vertices of the graph
        E               % Edges of the graph
        G               % Graph object
    end
    %
    methods
        function obj = RRT(qI, qG, n, del_q, O, qmin, qmax, robot, tolerance)
            % Constructor for RRT class
            % Inputs:
            %  qI: Initial Point [qI_1, qI_2, ... qI_dim]
            %  qG: Goal Point [qG_1, qG_2, ... qG_dim]
            %  n: Number of vertices to sample
            %  del_q: Step size for extending the tree
            %  O: Cell array of obstacles, each represented as a 2xN matrix [x; y]
            %  qmin: Minimum q_i value for each dimension
            %  qmax: Maximum q_i value for each dimension
            %  robot: Robot object
            %  tolerance: Tolerance for reaching the goal (Default: 3.0)
            % Outputs:
            %  obj: RRT object
            obj.qI = qI;
            obj.qG = qG;
            obj.dim = length(qI);
            obj.n = n;
            obj.del_q = del_q;
            obj.O = O;
            obj.qmin = qmin;
            obj.qmax = qmax;
            obj.robot = robot;
            if nargin < 9
                obj.tolerance = 3.0;
            else
                obj.tolerance = tolerance;
            end
            obj = obj.buildRRT();
        end
        %
        function obj = buildRRT(obj)
            % Build the RRT graph
            % Outputs:
            % obj: Updated RRT object
            %
            % Initialize the graph with the initial point
            obj.V = [obj.qI];
            % Initialize the edges
            obj.E = [];
            % Build the graph, keep sampling until n vertices are added
            i = 1;
            while i < obj.n
                % Sample a random point in the configuration space
                qrand = obj.qmin + rand(1, obj.dim) .* (obj.qmax - obj.qmin);
                % Find the nearest vertex in the graph
                [qnear, idx_near] = obj.Nearest_Vertex(qrand, obj.V);
                pnear = obj.robot.updateJointAngles(qnear).ee;
                % Extend the tree towards the random point
                qnew = obj.New_Conf(qnear, qrand);
                pnew = obj.robot.updateJointAngles(qnew).ee;
                % Check if the edge is in collision
                if ~obj.isInCollision([pnear; pnew])
                    % Add the new vertex to the graph
                    obj.V = [obj.V; qnew];
                    % Add the edge to the graph
                    obj.E = [obj.E; idx_near, size(obj.V, 1)];
                    i = i + 1;
                    % Check if the goal is reached
                    if norm(qnew - obj.qG) <= obj.tolerance
                        % Add the goal point to the graph
                        obj.V = [obj.V; obj.qG];
                        % Add the edge to the graph
                        obj.E = [obj.E; size(obj.V, 1) - 1, size(obj.V, 1)];
                        break;
                    end
                end
            end
            % Create a graph object
            obj.G = graph([obj.E(:,1); obj.E(:,2)], [obj.E(:,2); obj.E(:,1)]);
        end
        %
        function path = findPath(obj)
            % Find the path from the initial point to the goal point
            % Outputs:
            % path: Path from the initial point to the goal point
            % Find the indices of the start and goal points
            startIdx = find(ismember(obj.V, obj.qI, 'rows'));
            goalIdx = find(ismember(obj.V, obj.qG, 'rows'));
            % Find the shortest path in the graph
            [idx_path, ~] = shortestpath(obj.G, startIdx, goalIdx);
            % Extract the path from the graph
            path = obj.V(idx_path, :);
        end
        %
        function inCollision = isInCollision(obj, S)
            % Check if the line segment S is in collision with any obstacle
            % Inputs:
            % S: Line segment represented as a 2x2 matrix [x1, x2; y1, y2]
            % Outputs:
            % inCollision: Flag indicating if the line segment is in collision
            % Initialize the flag
            inCollision = false;
            % Check for collision with each obstacle
            for i = 1:length(obj.O)
                % Check if the line segment intersects with the polygon
                if obj.isintersect_linepolygon(S, obj.O{i})
                    inCollision = true;
                    return;
                end
            end
        end
    end
    %
    methods (Access = private)
        function b = isintersect_linepolygon(obj, S, Q)
            % Check if the line segment S intersects with the polygon Q
            % Inputs:
            % S: Line segment represented as a 2x2 matrix [x1, x2; y1, y2]
            % Q: Polygon represented as a 2xN matrix [x; y]
            % Outputs:
            % b: Flag indicating if the line segment intersects with the polygon
            % Extract the start and end points of the line segment
            p0 = S(1,:);
            p1 = S(2,:);
            %
            % Check if the line segment is a point
            if p0 == p1
                b = inpolygon(p0(1), p0(2), Q(1,:), Q(2,:));
                return;
            end
            %
            n = size(Q, 2);
            b = false;
            %
            % Check for intersection with each edge of the polygon
            for i = 1:n
                % Extract the start and end points of the edge
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
            % p0: Start point of line segment p0p1
            % p1: End point of line segment p0p1
            % q0: Start point of line segment q0q1
            % q1: End point of line segment q0q1
            % Outputs:
            % flag: Flag indicating if the line segments intersect
            % Calculate the direction vectors
            ds = p1 - p0;
            n1 = q1 - q0;
            n2 = p0 - q0;
            % Calculate the determinant
            ds_n_dot = ds(1) * n1(2) - ds(2) * n1(1);
            %
            % Check if the line segments are parallel
            if ds_n_dot == 0
                % Calculate the parameters for the intersection point
                tL = dot(n2, n1) / dot(n1, n1);
                tE = dot(n2 + ds, n1) / dot(n1, n1);
                flag = (min(tL, tE) <= 1) && (max(tL, tE) >= 0);
            else
                % Calculate the parameters for the intersection point
                tL = (n2(2) * n1(1) - n2(1) * n1(2)) / ds_n_dot;
                tE = (n2(2) * ds(1) - n2(1) * ds(2)) / ds_n_dot;
                flag = (tL >= 0) && (tL <= 1) && (tE >= 0) && (tE <= 1);
            end
        end
        %
        function [qnear, idx] = Nearest_Vertex(obj, qrand, V)
            % Find the nearest vertex in the graph to the random point
            % Inputs:
            % qrand: Random point in the configuration space
            % V: Vertices of the graph
            % Outputs:
            % qnear: Nearest vertex in the graph to the random point
            % idx: Index of the nearest vertex in the graph
            % Initialize the variables
            minDist = inf;
            idx = 0;
            for i = 1:size(V, 1)
                % Calculate the distance between the vertex and the random point
                d = norm(V(i,:) - qrand);
                % Update the minimum distance, nearest vertex and index
                if d < minDist
                    minDist = d;
                    qnear = V(i,:);
                    idx = i;
                end
            end
        end
        %
        function qnew = New_Conf(obj, qnear, qrand)
            % Extend the tree from qnear towards qrand
            % Inputs:
            % qnear: Nearest vertex in the graph to the random point
            % qrand: Random point in the configuration space
            % Outputs:
            % qnew: New vertex in the graph
            % Calculate the direction from qnear to qrand
            dir = (qrand - qnear) / norm(qrand - qnear);
            % Move a step of del_q from qnear toward qrand
            qnew = qnear + dir * obj.del_q;
        end
    end
end
