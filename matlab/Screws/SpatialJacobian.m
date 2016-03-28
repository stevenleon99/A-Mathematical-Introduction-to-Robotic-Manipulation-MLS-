function Jacobian = SpatialJacobian(varargin)
% Jacobian = SpatialJacobian([xi1,th1],[xi2,th2],...,g0)
% Return the nx6 Jacobian of the forward kinematic map for a set of revolute 
% and/or prismatics joint twists expressed as 6x1 vectors xi, displacements about those
% twists of scalar amount thi, and the initial position g0 of the end effector
% expresed as a 4x4 homogeneous transform.  

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

JsA = varargin{1};
  Js = JsA(1:6)';
 Ang = JsA(7);
g = TwistExp(Js', Ang);

for i = 2:numargin-1
   JsAi=varargin{i};
   Jsi = JsAi(1:6);
   Angi = JsAi(7);
   RigidAdjoint(g);
   xith = RigidAdjoint(g) * Jsi';

   Js;
   Js = [ Js, xith ];
   g = g * TwistExp(Jsi, Angi);
end

Jacobian = Js;

return;
