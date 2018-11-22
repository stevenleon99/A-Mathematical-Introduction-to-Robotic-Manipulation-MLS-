function theta = invk(g,soln)
% invk.m - inverse kinematics of Intelledex
% syntax: theta = invk(g,soln)
%     theta is a 6x1 vector of joint angles
%     g is a 4x4 frame
%     soln is an optional integer argument in 0,..,7 to pick solution
%
% DC Deno 9-26-91

% Could speed up considerably by:
%    1) compiling code with an optimizing compiler
%    2) compute directly inverse of 4x4 homogeneous matrix
%         with R^T and -R^Tp
%    3) compute directly exponentials of twists with Rodriques'
%         formula and other simple expressions
%    4) leave static definitions out of functions

% set decision tree array as 1x3 row vector of 1's and 0's
if nargin > 1,
    select = sdecide(soln);
else
    select = sdecide(0);
end

[l1,l2] = linklen;	% l1,l2 (link lengths)

% points on each twist axis
o0 = [-(l1+l2); 0; 0; 1];
o1 = o0;
o2 = o0;
o3 = [-l2; 0; 0; 1];
o4 = [0; 0; 0; 1];
o5 = o4;

% unit direction vectors of each twist axis
w0 = [0; 1; 0; 0];
w1 = [0; 0; 1; 0];
w2 = [0; -1; 0; 0];
w3 = [0; -1; 0; 0];
w4 = [0; -1; 0; 0];
w5 = [1; 0; 0; 0];

% compute 4x4 unit twists wrt initial frame location
xi0hat = twisthat(cross(o0,w0), w0);
xi1hat = twisthat(cross(o1,w1), w1);
xi2hat = twisthat(cross(o2,w2), w2);
xi3hat = twisthat(cross(o3,w3), w3);
xi4hat = twisthat(cross(o4,w4), w4);
xi5hat = twisthat(cross(o5,w5), w5);

% decompose into 2 parts:
%     cartesian location of end effector (theta 3,4,5)
%     orientation of "wrist" (actually at shoulder)

p012 = o0;
p45 = o4;
d = norm(g*p45 - p012);
theta3 = cp3(p45, p012, d, w3, o3, select(1));

b = inv(g)*p012;
q = expm(-xi3hat*theta3)*p012;
[theta4,theta5] = cp2(b, q, w4, w5, o4, select(2));

g012 = g*expm(-xi5hat*theta5)*expm(-xi4hat*theta4)*expm(-xi3hat*theta3);
[theta0,theta1] = cp2(w2, g012*w2, w0, w1, [0;0;0;0], select(3));

g2 = expm(-xi1hat*theta1)*expm(-xi0hat*theta0)*g012;
theta2 = cp1(w1, g2*w1, w2, [0;0;0;0]);

theta = real([theta0 theta1 theta2 theta3 theta4 theta5]);
