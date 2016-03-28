%(*
% * Example 2.6: Velocity of a two-link mechanism
% *
% * RMM, 4 Nov 93
% *
% *)

l0 = 10;
l1 = 10;
l2 = 10;
dth1 = 10/180*pi;
dth2 = 10/180*pi;

xiab = [0,0,0, 0,0,1];
Vab = xiab * dth1;
xibc = [l1,0,0, 0,0,1]; 
Vbc = xibc * dth2;

gab = TwistExp(xiab, th1) * RPToHomogeneous(eye(3), [0;0;l0]);

Vac = (Vab' + RigidAdjoint(gab) * Vbc')'
