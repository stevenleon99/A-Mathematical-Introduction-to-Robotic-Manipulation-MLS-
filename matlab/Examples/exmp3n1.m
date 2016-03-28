%(*
% * Example 3.1:  SCARA forward kinematics
% *
% * Richard M. Murray
% * 22 January 1992
% *
% * This file contains a portion of the file scara.m
% *)

%
% SCARA robot link lengths
%

l1 = 10;
l2 = 10;

%
% SCARA location of end effector tool frame initial position frame in global coordinates
%

gst0 = RPToHomogeneous(eye(3),[0;l1+l2;0]);

%
% SCARA joint angles
%

th1 = 0
th2 = 0
th3 = 0
th4 = 0

%
% Twist Axes for SCARA robot, starting from the base
%

xi1=RevoluteTwist([0;l1;0],[0;0;1]);
xi2=RevoluteTwist([0;l1;0],[0;0;1]);
xi3=RevoluteTwist([0;l1+l2;0],[0;0;1]);
xi4=RevoluteTwist([0;0;0],[0;0;1]);

%
% Forward kinematics map, resulting in a 4x4 HTM with tool frame
% in global coordinates, given the joint angles.
%

gst = ForwardKinematics([xi1,th1],[xi2,th2],[xi3,th3],[xi4,th4],gst0)
