
%
% SCARA robot forward kinematics.
% To use, change the joint angles th1, th2, th3, th4 and run.  
%   Inputs are the joint angles.
%   Output is a 4x4 HTM matrix of the tool frame position in global coordinates
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

%
% Spatial manipulator Jacobian
%

Js = SpatialJacobian([xi1,th1],[xi2,th2],[xi3,th3],[xi4,th4],gst0)

