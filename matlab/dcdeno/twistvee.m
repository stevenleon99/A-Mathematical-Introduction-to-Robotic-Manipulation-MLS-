function [v,w] = twistvee(xi)
% twistvee converts a 4x4 twist into its two 3x1 components
% [v,w] = twistvee(xi)
%
% DC Deno 9-26-91

v = xi(1:3,4);
w = [xi(3,2); xi(1,3); xi(2,1)];
