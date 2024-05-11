function vertex_trans = Transform(vertex,q)
% Argument:
% vertex: 2xn array
% q: 1x3: [xt,yt,theta]^T
% Return:
% vertex_trans: 2xn array: 2d transformed vertex based on q
translate = reshape(q(1:2),[2,1]);
theta = q(3);
R = [[cos(theta) -sin(theta)];...
     [sin(theta)  cos(theta)]];
vertex_rot = R * vertex;
vertex_trans = vertex_rot + translate;