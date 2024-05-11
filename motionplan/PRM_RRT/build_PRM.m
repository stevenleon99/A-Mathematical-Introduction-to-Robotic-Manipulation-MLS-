function [path, V, G] = build_PRM(q_init, q_goal, N, K, O, xmax, ymax)
% Initialize
path = [];
V = [];
G = Inf(N);
l = size(O, 2);
find = 0;

% Add initial q and goal q into V
V = [V q_init q_goal];
% Generate N - 2 number of collision free random nodes
while(size(V, 2) < N)
    xrand = rand * xmax;
    yrand = rand * ymax;
    qr = [xrand; yrand];
    % Check the collision of a random node
    is_col = 0;
    for i = 1 : l
        obs = O{i};
        if(inpolygon(xrand, yrand, obs(1, :), obs(2, :)))
            is_col = 1;
            break;
        end
    end
    if(is_col == 1)
        continue;
    end
    V = [V qr];
end

% For each node, find its nearest k neighbors
for i = 1 : size(V, 2)
    [qnear, q_index] = nearest_kneighbor(i, V, K);
    for j = 1 : size(qnear, 2)
        % Collision checking of a path
        b = 0;
        for k = 1 : l
            if(isintersect_linepolygon([qnear(:, j) V(:, i)], O{k}))
                b = 1;
                break;
            end
        end
        if(b ~= 0)
            continue;
        else
            % Add to the graph if there is no collision
            G(i, q_index(j)) = norm(V(:, i) - qnear(:, j));
            G(q_index(j), i) = norm(V(:, i) - qnear(:, j));
        end
    end
end

% Use Dijkstra algorithm to find a path
[find, lowest_cost, path_index] = Dijkstra(G, 1, 2);
% If a path is found, output the path
if(find == 1)
    for i = 1 : size(path_index, 2)
        path = [path V(:, path_index(i))];
    end
end

% Define nearest k neighbors function to find the nearest k neighbors 
function [qnear, q_index] = nearest_kneighbor(i, V, K)
% Initialize
q = V(:, i);
V(:, i) = [Inf; Inf];
qnear = [];
q_index = [];
qn = [];
% If the size of V is leq K, simply output V
if(size(V, 2) <= K)
    qnear = V;
    for i = 1 : size(V, 2)
        q_index = [q_index i];
    end
    return;
else
    for i = 1 : K
        qn = V(:, 1);
        min_dist = norm(q - qn);
        index = 1;
        for j = 1 : size(V, 2)
            qi = V(:, j);
            if(norm(q - qi) < min_dist)
                qn = qi;
                min_dist = norm(q - qi);
                index = j;
            end
        end
        % Add it to the nearest k neighbors
        qnear = [qnear qn];
        q_index = [q_index index];
        % Remove the node after adding
        V(:, index) = [Inf; Inf];
    end
end