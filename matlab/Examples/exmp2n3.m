%(*
% * Example 2.3:  (no title)
% *
% * RMM, 4 Nov 93
% *
% *)

%
% Problem setup
%
th = 10/180*pi
l1 = 10;
w = [0;0;1];
q = [0;l1;0];
xi = [(-AxisToSkew(w)*q)', w']

% Compute the exponential of the twist 
expxi = TwistExp(xi, th)

% Now compute the transformation between frames 
gab0 = RPToHomogeneous(eye(3), [0;l1;0])
gab = expxi * gab0
