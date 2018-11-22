function a_cross_b = cross(a,b)
% vector cross product, returns 4x1 direction vector
% cross(a,b) = a x b
%
% DC Deno 9-26-91

a_cross_b = [hat(a)*[b(1); b(2); b(3)]; 0];
