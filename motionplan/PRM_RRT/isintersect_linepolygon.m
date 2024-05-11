function b = isintersect_linepolygon(S, Q)

% Initialize
Q = [Q Q(:, 1)];
n = size(Q, 2);

p0 = [S(1, 1) S(2, 1)];
p1 = [S(1, 2) S(2, 2)];

% Check if S is a single point
if(p0 == p1)
    if(inpolygon(p0(1), p0(2), Q(1,:), Q(2,:)))
        b = 1;
        return;
    else
        b = 0;
        return;
    end
else
    te = 0;
    tl = 1;
    ds = p1 - p0;
    qp = [Q(1, n - 1) Q(2, n - 1)];
    for i = 1 : n - 1
        qi = [Q(1, i) Q(2, i)];
        qn = [Q(1, i + 1) Q(2, i + 1)];
        ep = qp - qi;
        ei = qn - qi;
        % Compute outward normal vector ni
        ni = cross(cross([ep 0], [ei 0]), [ei 0]);
        N = - dot(([p0 0] - [qi 0]), ni);
        D = dot([ds 0], ni);
        qp = qi;
        if(D == 0)
            if(N < 0)
                b = 0;
                return;
            end
            % When D is 0, t is not defined, so continue
            continue;
        end
        t = N / D;
        if(D < 0)
            te = max([te t]);
            if(te > tl)
                b = 0;
                return;
            end
        elseif(D > 0)
            tl = min([tl t]);
            if(tl < te)
                b = 0;
                return;
            end
        end
    end
    if(te <= tl)
        b = 1;
        return;
    else
        b = 0;
        return;
    end
end