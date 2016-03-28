%(*
% * Example 3.3:  SCARA kinematics with alternate base frame
% *
% * RMM, 4 Nov 93
% *
% *)
%
% SCARA robot link lengths
%

l1 = 10;
l2 = 10;

%
% SCARA location of end effector tool frame initial position frame in global coordinates
%

%
% SCARA joint angles
%

th1 = 0
th2 = 0
th3 = 0
th4 = 0

%(* twist axes for elbow robot, reference frame at end-effector *)
xi1 = RevoluteTwist([0;-l1-l2;0], [0;0;1]);	%(* base *)
xi2 = RevoluteTwist([0;-l2;0], [0;0;1]);	%(* elbow *)
xi3 = RevoluteTwist([0;0;0], [0;0;1]);		%(* wrist *)
xi4 = PrismaticTwist([0;0;0], [0;0;1]);	

gst0 = RPToHomogeneous(eye(3),[0;0;0]);

%
%(* Forward Kinematics *)
%
gst = ...
  TwistExp(xi1,th1) * TwistExp(xi2,th2) * ...
  TwistExp(xi3,th3) * TwistExp(xi4,th4) * gst0
