% input: 
% vertices - 2x3 array of the coordinates of the triangle vertices
% pt - 2x1 array of the coordinate of the point
% ---
% return:
% cloest_pt - 2x1 array of the coordinate of the cloest point

function [cloest_pt, Q] = CloestPointOnTriangleToPoint(vertices, pt)
    narginchk(2, 2);
    cloest_pt = [];
    Q = [];
    if ~(size(vertices, 1) == 2) || ~(size(vertices, 2) == 3)
        disp("the input triangle vertices with wrong dimension");
    end
    if ~(size(pt, 1) == 2) || ~(size(pt, 2) == 1)
        disp("the input pt vertices with wrong dimension");
    end

    % sort the scenario
    % pointA
    if  ( (pt-vertices(:, 1))' * (vertices(:, 2)-vertices(:, 1)) ) <=0 && ...
            ( (pt-vertices(:, 1))' * (vertices(:, 3)-vertices(:, 1)) ) <=0
        disp("closest pt is A")
        cloest_pt = vertices(:, 1);
        return
    end
    % pointB
    if  ( (pt-vertices(:, 2))' * (vertices(:, 1)-vertices(:, 2)) ) <=0 && ...
            ( (pt-vertices(:, 2))' * (vertices(:, 3)-vertices(:, 2)) ) <=0
        disp("closest pt is B")
        cloest_pt = vertices(:, 2);
        return
    end
    % pointC
    if  ( (pt-vertices(:, 3))' * (vertices(:, 1)-vertices(:, 3)) ) <=0 && ...
            ( (pt-vertices(:, 3))' * (vertices(:, 2)-vertices(:, 3)) ) <=0
        disp("closest pt is C")
        cloest_pt = vertices(:, 3);
        return
    end
    % edge AB
    if ( (cross(cross([vertices(:, 3)-vertices(:, 2);0], [vertices(:, 1)-vertices(:, 2);0]), [vertices(:, 1)-vertices(:, 2); 0]))' * ([pt-vertices(:, 2); 0]) ) >= 0 && ...
           ( (pt-vertices(:, 1))' * (vertices(:, 2)-vertices(:, 1)) ) >= 0 && ...
           ( (pt-vertices(:, 2))' * (vertices(:, 1)-vertices(:, 2)) ) >= 0
        disp("closest pt is on AB")
        cloest_pt = pointOnLine(vertices(:, 1), vertices(:, 2), pt);
        Q = [vertices(:, 1), vertices(:, 2)];
        return
        
    end
    % edge BC
    if ( (cross(cross([vertices(:, 1)-vertices(:, 3);0], [vertices(:, 2)-vertices(:, 3);0]), [vertices(:, 2)-vertices(:, 3); 0]))' * ([pt-vertices(:, 3); 0]) ) >= 0 && ...
           ( (pt-vertices(:, 2))' * (vertices(:, 3)-vertices(:, 2)) ) >= 0 && ...
           ( (pt-vertices(:, 3))' * (vertices(:, 2)-vertices(:, 3)) ) >= 0
        disp("closest pt is on BC")
        cloest_pt = pointOnLine(vertices(:, 2), vertices(:, 3), pt);
        Q = [vertices(:, 2), vertices(:, 3)];
        return
        
    end
    % edge CA
    if ( (cross(cross([vertices(:, 2)-vertices(:, 1);0], [vertices(:, 3)-vertices(:, 1);0]), [vertices(:, 3)-vertices(:, 1); 0]))' * ([pt-vertices(:, 1); 0]) ) >= 0 && ...
           ( (pt-vertices(:, 1))' * (vertices(:, 3)-vertices(:, 1)) ) >= 0 && ...
           ( (pt-vertices(:, 3))' * (vertices(:, 1)-vertices(:, 3)) ) >= 0
        disp("closest pt is on CA")
        cloest_pt = pointOnLine(vertices(:, 3), vertices(:, 1), pt);
        Q = [vertices(:, 3), vertices(:, 1)];
        return
        
    end
    
end



function D = pointOnLine(A, B, C)

% Calculate vectors AB and AC
AB = B - A;
AC = C - A;

% Project AC onto AB to find the scalar projection magnitude of AD
AD_scalar = dot(AC, AB) / dot(AB, AB);

% Find the closest point D on the line
D = A + AD_scalar * AB;

% Check if D is within the segment AB
if AD_scalar < 0
    D = A; % D is beyond A, so A is the closest point
elseif AD_scalar > 1
    D = B; % D is beyond B, so B is the closest point
end

% D now holds the closest point on AB to C

end