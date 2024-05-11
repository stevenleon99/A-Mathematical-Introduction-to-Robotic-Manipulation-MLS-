% Test Case
% A = [1 1 3; -1 0.5 -1];
A = [1 2 3; -1 0.5 -1];
% A = [1 0 -1; -1 0.5 -1];
B1 = [1 3 3 1; 1 1 3 3];

distance = GJKalg_2D(A, B1);
fprintf("the distance is -> %d", distance);