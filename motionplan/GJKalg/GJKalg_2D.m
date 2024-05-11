% input:
% A - 2xna array of the coordinates of the vertices of Robot
% B - 2xnb array of the coordinates of the vertices of Obstacle
% output:
% distance - distance between A and B

function distance = GJKalg_2D(A, B1)

    B = {B1}; % change input as array
    q = [0; 0; 0*pi/180]; % no rotation to A
    CB = fn_c_obstacles(B, A, q); % return (X Y) -> CB{1}
    distance = -1;
    
    % display the Minkowski difference
    figure(1)
    hold on 
    patch(CB{1}(1,:),CB{1}(2,:),'g-');
    patch(A(1,:),A(2,:),'r-');
    patch(B1(1,:),B1(2,:),'r-');
    axis equal
    
    dim = 2;
    Q = [CB{1}(1,3:5); CB{1}(2,3:5)];
    
    while 1
        [P, Q] = CloestPointOnTriangleToPoint(Q, [0;0]);
        
        if (P(1,1) == 0 && P(2,1) == 0)
            distance = 0;
            return;
        end
        
        if (size(Q, 2) == 0)
            distance = sqrt(sum(power(P, 2), 1));
            return
        end

        [V, maxDot] = findSupportingPoint(CB{1}, -P);
        maxDot_ = dot(P, -P);
        fprintf("maxDot: %f, maxDot_: %f, equal: %d \n", maxDot, maxDot_, (maxDot-maxDot_ < eps))
        if (maxDot-maxDot_) < eps
            distance = sqrt(sum(power(P, 2), 1));
            return 
        end
        Q = [Q, V];

    end
    
end

function [supportingPoint, maxDot] = findSupportingPoint(shape, direction)
    % Initialize the index of the vertex with the maximum dot product
    maxIndex = 1;
    % Initialize the maximum dot product value
    maxDot = dot(shape(:,1), direction);
    
    % Iterate through all vertices to find the max dot product
    for i = 2:size(shape,2)
        currentDot = dot(shape(:,i), direction);
        if currentDot > maxDot
            maxDot = currentDot;
            maxIndex = i;
        end
    end
    
    % Return the vertex with the maximum dot product as the supporting point
    supportingPoint = shape(:,maxIndex);
end