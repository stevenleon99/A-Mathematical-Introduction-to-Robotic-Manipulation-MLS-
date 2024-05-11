function [path, V, E] = build_RRT(q_init, q_goal, N, delta_q, O, xmax, ymax, tolerance)
% Initialize
path = [];
V = [];
E = {};
l = size(O, 2);
find = 0;

V = [V q_init];
k = 1;
while(1)
    p = rand;
    % Generate the random q with goal bias
    if(p < 0.05)
        qnew = q_goal;
        qnear = nearest_neighbor(qnew, V);
    else
        xrand = rand * xmax;
        yrand = rand * ymax;
        qr = [xrand; yrand];
        qnear = nearest_neighbor(qr, V);
        qnew = qnear + delta_q * (qr - qnear) / norm(qr - qnear);
    end
    % Collision checking of the edge
    b = 0;
    for i = 1 : l
        if(isintersect_linepolygon([qnear qnew], O{i}))
            b = 1;
            break;
        end
    end
    if(b ~= 0)
        continue
    else
        % Add the node and the edge if there is no collision
        V = [V qnew];
        E{k} = [qnear qnew];  
    end
    k = k + 1;
    % Stop if q_goal is reached
    if(qnew == q_goal)
        find = 1;
        break;
    end
    % Stop if maximnm number of nodes is reached
    if(k >= N)
        break;
    end
end

% If a path is found, perform backward search to reconstruct the path
if(find == 1)
    n = size(E, 2);
    qp = q_goal;
    path = [path qp];
    while(qp ~= q_init)
        for i = 1 : n
            ep = E{i};
            if(ep(:, 2) == qp)
                break;
            end
        end
        qp = ep(:, 1);
        path = [path qp];
    end
    path = fliplr(path);
end

% Define nearest neighbor function to find the nearest neighbor of a node
function qnear = nearest_neighbor(q, V)
qnear = V(:, 1);
min_dist = norm(q - qnear);
n = size(V, 2);
for i = 1 : n
    qi = V(:, i);
    if(norm(q - qi) < min_dist)
        qnear = qi;
        min_dist = norm(q - qi);
    end
end