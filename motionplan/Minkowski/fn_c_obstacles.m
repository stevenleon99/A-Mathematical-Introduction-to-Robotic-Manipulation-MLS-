function C_obs = fn_c_obstacles(obs_vertices, A0_vertices, q)
n = size(obs_vertices, 2);
A0 = [cos(q(3)) -sin(q(3)); sin(q(3)) cos(q(3))] * A0_vertices;

% Sort the -A(0)
minusA0_sort = ccwsort(-A0);

C_obs = {};
for i = 1 : n
    obs = obs_vertices{i};
    % Sort the O
    obs_sort = ccwsort(obs);
    % Compute Cobs using Minkowski Sum
    C_obs{i} = Minkowski_Sum(minusA0_sort, obs_sort);
end
end

% Sort Function
function Asort = ccwsort(A)
n = size(A, 2);
% Select the smallest y-coordinate
ymin = A(2, 1);
index = 1;
for i = 2 : n
    if(A(2, i) < ymin)
        ymin = A(2, i);
        index = i;
    elseif(A(2, i) == ymin && A(1, i) < A(1, index))
        ymin = A(2, i);
        index = i;
    end
end

% Move it to the center
A_origin = A - mean(A, 2);

% Sort in CCW
angle = atan2(A_origin(2, index), A_origin(1, index));
A_angle = [];
for i = 1 : n
    A_angle(i) = atan2(A_origin(2, i), A_origin(1, i)) - angle;
    if(A_angle(i) < 0)
        A_angle(i) = A_angle(i) + 2 * pi;
    end
end
[~, A_order] = sort(A_angle);

Asort = [];
for i = 1: n
    Asort(:, i) = A(:, A_order(i));
end
end

% Minkowski Sum Function
function C = Minkowski_Sum(A, B)
i = 1;
j = 1;

n = size(A, 2);
m = size(B, 2);

A = [A A(:, 1) A(:, 2)];
B = [B B(:, 1) B(:, 2)];
C = [];

while(size(C, 2) < n + m)
    C = [C A(:, i)+B(:, j)];
    % Using cross product to campare the angles
    cross = (A(1, i + 1)- A(1, i))*(B(2, j + 1)- B(2, j)) - (B(1, j + 1)- B(1, j))*(A(2, i + 1)- A(2, i));
    if(cross > 0)
        i = i + 1;
    elseif(cross < 0)
        j = j + 1;
    else
        i = i + 1;
        j = j + 1;
    end
    if(i > n + 1 || j > m + 1)
        break
    end
end
end