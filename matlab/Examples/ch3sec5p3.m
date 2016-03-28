%(*
% * 5.3 Four-bar linkage
% *
% * Richard M. Murray
% * 6 February 1994
% *
% * Kevin Otto
% * 6 Oct 2011
% *
% *)


%
% Link lengths
%
r = 1;
h = 5;
l1 = 2;
l2 = 3;

%
% Change these angles to move the linkage
%

th11 = 0
th12 = 0
th21 = 0
th22 = 0

% (* Describe twists for each half of the four-bar *)
xi11 = RevoluteTwist([-r;0;0], [0;0;1]);
xi12 = RevoluteTwist([-r;l1;0], [0;0;1]);

xi21 = RevoluteTwist([r;h;0], [0;0;1]);
xi22 = RevoluteTwist([r;h+l2;0], [0;0;1]);

% (* Transformation between base and tool frams *)
gst0 = RPToHomogeneous(eye(3), [0;l1;0]);

% (* Expand out the two sides *)
g1 = TwistExp(xi11,th11) * TwistExp(xi12,th12) * gst0
g2 = TwistExp(xi21,th21) * TwistExp(xi22,th22) * gst0

% (* Now calculate out the Jacobian of the structure equations *)
xi12p = RigidAdjoint(TwistExp(xi11,th11)) * xi12' 
xi22p = RigidAdjoint(TwistExp(xi21,th21)) * xi22' 
