%(*
% * Example 2.2:  Twist coordinates for rotation about an axis
% *
% * RMM, 4 Nov 93
% *
% *)

%(*
% * First part: compute the twist coords for absolute transformation
% *
% *)

%
%(* Problem set up: construct rigid body transformation *)
%
l1 = 10;
l2 = 10;
w = [0;0;1];
alpha = 10*pi/180

Rab = SkewExp(w, alpha);
pab = [-l2 * sin(alpha); l1 + l2*cos(alpha); 0];
gab = RPToHomogeneous(Rab, pab);

%
%(* Figure out the vector v *)
%
A = (eye(3) - Rab) * AxisToSkew(w) + w * w' * alpha;
Ainv = A^-1;
v = Ainv * pab;

%
%(* Compute the rigid motion associated with a twist *)
%
xi = [l1,0,0,0,0,1];
expxi = TwistExp(xi, th)
gab0 = RPToHomogeneous(eye(3), [0;l1;0]);

%
%(* figure out the twist coordinates *)
%
xi = [v(1), v(2), v(3), w(1), w(2), w(3)]

%
%(*
% * Second part: compute the twist coords for relative transformation
% *
% *)
%

gabz = RPToHomogeneous(SkewExp(w, 0), [-l2 * sin(0); l1 + l2*cos(0); 0]);

grel = gab * RigidInverse(gabz);
xirel = RigidTwist(grel)
