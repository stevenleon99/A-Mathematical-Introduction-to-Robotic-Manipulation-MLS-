function g = fwdk(theta)
% fwdk.m - forward kinematics of Intelledex
% g = fwdk(theta), where theta is a 6x1 vector of joint angles
%     g is 4x4 frame
%     l1,l2 (link lengths)
%
% DC Deno 9-26-91

[l1,l2] = linklen;

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

% product of exponentials formula
g =   expm(th0*xi0hat)*expm(th1*xi1hat)*expm(th2*xi2hat);
g = g*expm(th3*xi3hat)*expm(th4*xi4hat)*expm(th5*xi5hat);
