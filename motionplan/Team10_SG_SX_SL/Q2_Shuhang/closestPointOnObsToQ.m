function [closest_pt, minDist] = closestPointOnObsToQ(obs, q)
    % This helper function will return the cloesest point on obstacle to
    % point q
    % Read vertex
    v1 = obs(:,1);
    v2 = obs(:,2); 
    v3 = obs(:,3); 
    v4 = obs(:,4);
    
    edges = {v2-v1, v3-v2, v4-v3, v1-v4};
    verticesSet = {v1, v2, v3, v4};
    
    % Initialize variables
    minDist = inf;
    closest_pt = [0; 0];
    
    % Check each vertex region
    for i = 1:4
        if i == 1
            prev = 4;
        else
            prev = i-1;
        end
        
        p1 = verticesSet{i};
        p2 = verticesSet{mod(i,4)+1};
        edge = edges{i};
        prevEdge = edges{prev};
        
        % vector from q to vertex
        temp = q - p1;
        if dot(temp, edge) <= 0 && dot(temp, -prevEdge) <= 0
            % Check the distance between vertex and q
            if norm(temp) < minDist
                minDist = norm(temp);
                closest_pt = p1;
            end
        else
            % Project point onto edge
            t = max(0, min(1, dot(temp, edge) / dot(edge, edge)));
            proj = p1 + t * edge;
            dist = norm(q - proj);
            if dist < minDist
                minDist = dist;
                closest_pt = proj;
            end
        end
    end
end