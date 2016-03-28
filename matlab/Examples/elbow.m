

%
%  elbow.m - Kinematics for 6R elbow manipulator
%
% 

%
%  Written by Dr. Kevin Otto
%  See LICENSE.txt for copyright info.
%
%
%  Robust Systems and Strategy LLC
%  otto@robuststrategy.com
%  Version 1.0
%

%
% Link lengths
%
l0 = 10;
l1 = 10;
l2 = 10;

%
% Joint angles.  Change these to move the end effector
%
th1 = 0;
th2 = 0;
th3 = 0;
th4 = 0;
th5 = 0;
th6 = 0;

% twist axes for SCARA robot, reference frame at base 
xi1 = ScrewToTwist(0, [0;0;l0], [0;0;1]);       %  base 
xi2 = ScrewToTwist(0, [0;0;l0], [-1;0;0]);
xi3 = ScrewToTwist(0, [0;l1;l0], [-1;0;0]);     % elbow 
xi4 = ScrewToTwist(0, [0;l1+l2;l0], [0;0;1]);	% wrist 
xi5 = ScrewToTwist(0, [0;l1+l2;l0], [-1;0;0]);
xi6 = ScrewToTwist(0, [0;l1+l2;l0], [0;1;0]);

g0 = RPToHomogeneous(eye(3), [0;l1+l2;l0]);

%(* Forward Kinematics *)
g = ...
  TwistExp(xi1,th1) * TwistExp(xi2,th2) * ...
  TwistExp(xi3,th3) * TwistExp(xi4,th4) * ...
  TwistExp(xi5,th5) * TwistExp(xi6,th6) * g0


