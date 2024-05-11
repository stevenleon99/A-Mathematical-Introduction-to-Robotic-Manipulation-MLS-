function [pathfound, lowest_cost, shortest_path]=Dijkstra(graph, n_init, n_goal)

%Initialize
lowest_cost = Inf;
shortest_path = [];

%Compute the total number of nodes
n_num = size(graph,1);

%Initialize temporary distances dist and the closest neighbor prev
for n = 1 : n_num
    dist(n) = Inf;
    prev(n) = NaN;
end

%Initialize the Initial Node
dist(n_init) = 0;

%Create Unvisited Set U
U = 1 : n_num;

%Repeat while Goal Node has not been visited
while find(U == n_goal)
    min_dist = Inf;
    C = NaN;

    %Select the node with the smallest temporary distance
    for i = 1 : size(U, 2)
        if dist(U(i)) < min_dist
            min_dist = dist(U(i));
            C = U(i);
        end
    end

    %If C is NaN then there is no path
    if isnan(C)
        pathfound = 0;
        return;
    end 

    %Remove the node from Unvisited Set U
    U(U == C) = [];

    %Update the distance and the closest neighbor of neighbors
    for i = 1 : n_num
        alt = Inf;
        if graph(C, i) ~= Inf && graph(C, i) > 0
            alt = dist(C) + graph(C, i);
            if alt < dist(i)
                dist(i) = alt;
                prev(i) = C;
            end
        end
    end
end

pathfound = 1;

%Output lowest cost to Goal Node
lowest_cost = dist(n_goal);

%Construct and output the shortest path based on prev
cur_node = n_goal;
path(1) = n_goal; 
while cur_node ~= n_init
    cur_node = prev(cur_node);
    path(end + 1) = cur_node;
end
shortest_path = fliplr(path);
return