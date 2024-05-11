function V = potential_field(qI, qG, CB)
V = [];
q = qI;
V = [V q];
a = 0.005;
b = [5000; 500];
Uatt = [Inf; Inf];
Urep = [Inf; Inf];
while norm(Uatt + Urep) > 0.0001
    a = a * 1.005;
    Uatt = [0; 0];
    Urep = [0; 0];

    % Compute Uatt
    Uatt = q - qG;

    % Compute Urep
    for i = 1 : size(CB, 2)
        cobs = CB{i};
        [x, y] = dist(q, cobs);
        d = norm([x y]);
        if d > 100
            continue;
        else
            Urep = Urep + b(i) * [x * (0.01 - 1/d) / power(d, 3); y * (0.01 - 1/d) / power(d, 3)];
        end
    end
    q = q - a * (Uatt + Urep);
    V = [V q];
end

V = [V qG];


function [x, y] = dist(q, cobs)
x = 0;
y = 0;
xmax = max(cobs(1, :));
xmin = min(cobs(1, :));
ymax = max(cobs(2, :));
ymin = min(cobs(2, :));

if q(1) > xmax && q(2) > ymax
    x = q(1) - xmax;
    y = q(2) - ymax;
elseif q(1) > xmax && q(2) < ymin
    x = q(1) - xmax;
    y = q(2) - ymin;
elseif q(1) < xmin && q(2) > ymax
    x = q(1) - xmin;
    y = q(2) - ymax;
elseif q(1) < xmin && q(2) < ymin
    x = q(1) - xmin;
    y = q(2) - ymin;
elseif q(1) < xmax && q(1) > xmin && q(2) > ymax
    x = 0;
    y = q(2) - ymax;
elseif q(1) < xmax && q(1) > xmin && q(2) < ymin
    x = 0;
    y = q(2) - ymin;
elseif q(1) > xmax && q(2) < ymax && q(2) > ymin
    x = q(1) - xmax;
    y = 0;
elseif q(1) < xmin && q(2) < ymax && q(2) > ymin
    x = q(1) - xmin;
    y = 0;
end