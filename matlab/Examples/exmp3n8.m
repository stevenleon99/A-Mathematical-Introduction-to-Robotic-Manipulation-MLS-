%(*
% * Example 3.8:  Jacobian for a SCARA robot
% *
% * Richard M. Murray
% * 22 January 1992
% *
% * This file contains a portion of the file scara.m
% *)

exmp3n1			% (* forward kinematics for SCARA *)

%(* Spatial Jacobian *)


  xi1
  (RigidAdjoint(TwistExp(xi1,th1))* xi2')'
  (RigidAdjoint(TwistExp(xi1,th1) * TwistExp(xi2,th2)) * xi3')' 
  (RigidAdjoint(TwistExp(xi1,th1) * TwistExp(xi2,th2) * TwistExp(xi3,th3)) * xi4')' 


Js = [ ...
  xi1;
  (RigidAdjoint(TwistExp(xi1,th1))* xi2')';
  (RigidAdjoint(TwistExp(xi1,th1) * TwistExp(xi2,th2)) * xi3')' ;
  (RigidAdjoint(TwistExp(xi1,th1) * TwistExp(xi2,th2) * TwistExp(xi3,th3)) * xi4')' ]'
