%(*
% * Example 2.1:  Rotation about a line
% *
% * RMM, 4 Nov 93
% *
% *)

th = 0

Rab = SkewExp([0;0;1], th);
pab = [0;l1;0];

gab = RPToHomogeneous(Rab, pab)

