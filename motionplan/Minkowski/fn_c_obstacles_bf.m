function C_obs = fn_c_obstacles_bf(obs_vertices, A0_vertices, q)
l = size(obs_vertices, 2);
n = size(A0_vertices, 2);
A0 = [cos(q(3)) -sin(q(3)); sin(q(3)) cos(q(3))] * A0_vertices;

C_obs = {};
for i = 1 : l
    obs = obs_vertices{i};
    m = size(obs, 2);
    cobsp = [];
    cobs = [];
    for j = 1 : m
        for k = 1 : n
            cobsp = [cobsp obs(:, j) - A0(:, k)];
        end
    end
    cobs_index = convhull(transpose(cobsp));
    p = size(cobs_index, 1);
    for j = 1 : p - 1
        cobs(:, j) = cobsp(:,cobs_index(j));
    end
    C_obs{i} = cobs;
end
end