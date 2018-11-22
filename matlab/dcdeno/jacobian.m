function j = jacobian(theta)
% Jacobian matrix determination of Intelledex
%    theta is vector of joint angles/displacements
%
% DC Deno 9-26-91

[l1,l2] = linklen;	% l1,l2 (link lengths)

% rename input angle arguments
th0 = theta(1);
th1 = theta(2);
th2 = theta(3);
th3 = theta(4);
th4 = theta(5);
th5 = theta(6);

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

% compute partial rigid body transformations of each link
g0 = eye(4);
g1 = g0*expm(th0*xi0hat);
g2 = g1*expm(th1*xi1hat);
g3 = g2*expm(th2*xi2hat);
g4 = g3*expm(th3*xi3hat);
g5 = g4*expm(th4*xi4hat);

% compute 4x4 unit twists in spatial coordinates at current config, theta
xi0hatp = g0*xi0hat*inv(g0);
xi1hatp = g1*xi1hat*inv(g1);
xi2hatp = g2*xi2hat*inv(g2);
xi3hatp = g3*xi3hat*inv(g3);
xi4hatp = g4*xi4hat*inv(g4);
xi5hatp = g5*xi5hat*inv(g5);

%
[v0, w0] = twistvee(xi0hatp); xi0 = [v0; w0];
[v1, w1] = twistvee(xi1hatp); xi1 = [v1; w1];
[v2, w2] = twistvee(xi2hatp); xi2 = [v2; w2];
[v3, w3] = twistvee(xi3hatp); xi3 = [v3; w3];
[v4, w4] = twistvee(xi4hatp); xi4 = [v4; w4];
[v5, w5] = twistvee(xi5hatp); xi5 = [v5; w5];

% assemble jacobian matrix by columns, the 6x1 twist coordinates of above
j = [xi0 xi1 xi2 xi3 xi4 xi5];
