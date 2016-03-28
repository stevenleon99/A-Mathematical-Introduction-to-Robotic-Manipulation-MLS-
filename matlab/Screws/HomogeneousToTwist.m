function Twist = HomogeneousToTwist(g)
% Twist = HomogeneousToTwist(g)
% Calculates the 6x1 twist vector from a 4x4 homogenous transform HTM g.
% Assumes the 4x4 HTM is composes of a skew-symetric rotation matrix R
% and a position vector p.  
% Uses SkewToAxis to find the twist direction w

%
%  Written by Dr. Kevin Otto
%  See LICENSE.txt for copyright info.
%
%
%  Robust Systems and Strategy LLC
%  otto@robuststrategy.com
%  Version 1.0
%

error(nargchk(1, 1, nargin))
if( (size(g,1) ~= 4) | (size(g,2) ~= 4) )
   error('Screws:HomogeneousToTwist:Input','Input matrix is not a 4x4 matrix.\n');
end

R = SkewToAxis(g(1:3,1:3));
Twist = [ g(1,4), g(2,4), g(3,4), R(1), R(2), R(3) ];

return;
