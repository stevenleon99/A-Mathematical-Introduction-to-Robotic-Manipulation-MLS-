function [V, G, n_init, n_goal, Edge] = vertical_cell_decomposition(qI, qG, CB, bounds)
Ep = [];
S = [];
OE = [];
BE = [];
for i = 1 : size(CB, 2)
    cobs = CB{1, i};
    Ep = [Ep cobs];
    cobs = [cobs cobs(:, 1)];
    for j = 1 : size(cobs, 2) - 1
        if cobs(1, j) < cobs(1, j + 1)
            OE = [OE [cobs(:, j); cobs(:, j + 1)]];
        else
            OE = [OE [cobs(:, j + 1); cobs(:, j)]];
        end
    end
end
bounds = [bounds bounds(:, 1)];
for i = 1 : size(bounds, 2) - 1
    if bounds(1, i) < bounds(1, i + 1)
        BE = [BE [bounds(:, i); bounds(:, i + 1)]];
    else
        BE = [BE [bounds(:, i + 1); bounds(:, i)]];
    end
end
Ep = [Ep bounds];
d = [];
for i = 1 : size(Ep, 2)
    for j = 1 : size(CB, 2)
        cobs = CB{1, j};
        [in, on] = inpolygon(Ep(1, i), Ep(2, i), cobs(1, :), cobs(2, :));
        if in == 1 && on == 0
            d = [d i];
        end
    end
end

for i = 1 : size(d, 2)
    Ep(:, d(i)) = [];
end

S = [OE BE];
[~, index] = sort(S(1, :));
S = S(:, index);
Ex = Ep(1, :);
Ex = unique(Ex);
L = [];
V = [];

for i = 1 : size(Ex, 2)
    L = [L S(:, S(1, :) == Ex(i))];
    [~, index] = sort(L(4, :));
    L = L(:, index);
    Ey = [];
    node = [];

    for j = 1 : size(L, 2)
        if L(1,j) == L(3,j)
            continue
        else
            Ey = [Ey interp1(L([1, 3], j), L([2, 4], j), Ex(i))];
        end 
    end

    Ey = unique(Ey);
    ym = (Ey(1: end - 1) + Ey(2 : end)) / 2;
    p = [Ex(i) * ones(1, numel(ym)); ym];
    node = p(:, arrayfun(@(n) is_Cfree(p(:, n), CB), 1 : size(p, 2)));
    
    V = [V node];
    L(:, L(3, :) == Ex(i)) = [];
end

V = add_unique_node(V, qI);
V = add_unique_node(V, qG);
[~, index] = sort(V(1, :));
V = V(:, index);
Vx = unique(V(1,:));
Ec = [];
Edge = [];
weights = [];

for q = 1 : size(Vx, 2) - 1
    cur = V(:, V(1, :) == Vx(1, q));
    nex = V(:, V(1, :) == Vx(1, q + 1));
    [Edge, Ec, weights] = add_valid_edges(Edge, Ec, weights, cur, nex, V, OE);
end

n = size(V,2);
G = build_weighted_adj_matrix(n, Ec, weights);
n_init = getindex(qI, V);
n_goal = getindex(qG, V);
end

% Helper functions

function b = is_Cfree(q, O)
b = true;
for i = 1 : size(O, 2)
    cobs = O{1, i};
    if inpolygon(q(1), q(2), cobs(1, :), cobs(2, :))
        b = false;
        return
    end
end
end

function b = isintersect_linecobs(l1, O)
b = false;
for i = 1 : size(O, 2)
    l2 = O(:,i);
    if isintersect_lines(l1,l2)
        b = true;
        return
    end
end 
end

function b = isintersect_lines(l1,l2)
lx = [l1(1, 1) l1(3, 1) l2(1, 1) l2(3, 1)];
ly = [l1(2, 1) l1(4, 1) l2(2, 1) l2(4, 1)];
d1 = det([1, 1, 1; lx(1), lx(2), lx(3); ly(1), ly(2), ly(3)]) * det([1, 1, 1; lx(1), lx(2), lx(4); ly(1), ly(2), ly(4)]);
d2 = det([1, 1, 1; lx(1), lx(3), lx(4); ly(1), ly(3), ly(4)]) * det([1, 1, 1; lx(2), lx(3), lx(4); ly(2), ly(3), ly(4)]);

if(d1 <= 0 && d2 <= 0)
    b = true;
else
    b = false;
end
end

function i = getindex(q, V)
[~, id] = find(V == q);
if size(id, 1) == 2
    i = id(1);
else
    index = unique(id);
    count = hist(id,index);
    indexvalue = (count ~= 1);
    i = index(indexvalue);
end
end

function V = add_unique_node(V, node)
if ~any(all(V == node, 1))
    V = [V, node];
end
end

function [E, Ec, weights] = add_valid_edges(E, Ec, weights, cur, nex, V, O)
for s = 1 : size(cur, 2)
    for t = 1 : size(nex, 2)
        line = [cur(:, s); nex(:, t)];
        if ~isintersect_linecobs(line, O)
            E = [E line];
            Ec = [Ec [getindex(cur(:, s), V); getindex(nex(:, t), V)]];
            weights = [weights norm(cur(:, s) - nex(:, t))];
        end
    end
end
end

function G = build_weighted_adj_matrix(n_nodes, EI, weights)
G = ones(n_nodes) * Inf;
G(sub2ind(size(G), 1 : n_nodes, 1 : n_nodes)) = 0;
for i = 1:size(EI,2)
    G(EI(1,i),EI(2,i)) = weights(i);
    G(EI(2,i),EI(1,i)) = weights(i);
end
end