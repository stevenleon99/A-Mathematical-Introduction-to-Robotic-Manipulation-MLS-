function Tform = ForwardKinematics(varargin)
% Tform = ForwardKinematics([xi1,th1],[xi2,th2],...,g0)
% Return the 4x4 homongeneous transform forward kinematic map for a set of revolute 
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
    error('Screws:ForwardKinematics:Input','Number of inputs must be > 1.\n');
end
%if mod(numargin,2) == 0
%    error('Screws:ForwardKinematics:Input','Number of inputs must be an odd number.\n');
%end

xith = varargin{1};
g = TwistExp(xith(1:6), xith(7));

for i = 2:numargin-1
   xith = varargin{i};
   g = g * TwistExp(xith(1:6), xith(7));
end

Tform = g * varargin{numargin};

return;
