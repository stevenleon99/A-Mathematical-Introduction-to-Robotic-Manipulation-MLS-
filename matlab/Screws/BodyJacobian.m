function Jacobian = BodyJacobian(varargin)
% Jacobian = BodyJacobian([xi1,th1],[xi2,th2],...,g0)
% Return the nx6 Jacobian of the forward kinematic map in end-effector coordinates
% for a set of revolute and/or prismatics joint twists expressed as 6x1 vectors xi, 
% displacements about those twists of scalar amount thi, and the initial position g0 
% of the end effector expresed as a 4x4 homogeneous transform.  

%
%  Written by Dr. Kevin Otto
%  See LICENSE.txt for copyright info.
%
%
%  Robust Systems and Strategy LLC
%  otto@robuststrategy.com
%  Version 1.0
%

numargin = size(varargin,2);

if numargin < 2
    error('Screws:SpatialJacobian:Input','Number of inputs must be > 1.\n');
end

JbA = varargin{1};
  Jb = JbA(1:6)';
 Ang = JbA(7);
g = TwistExp(Jb', Ang);

for i = 2:numargin-1
   JbAi=varargin{i};
   Jbi = JbAi(1:6);
   Angi = JbAi(7);
   RigidAdjoint(g);
   xith = RigidAdjoint(RigidInverse(g)) * Jbi';

   Jb = [ xith, Jb ];
   g = g * TwistExp(Jbi, Angi);
end

Jacobian = Jb;

return;
