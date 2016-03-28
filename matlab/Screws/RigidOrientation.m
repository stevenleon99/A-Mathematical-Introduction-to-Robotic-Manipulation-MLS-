function Rotate = RigidOrientation(g)
% Rotate = RigidOrientation(g)
% Extracts the 3x3 rotation matrix from a 4x4 homogenous transform.  

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
if( (size(g,1)~=4) | (size(g,2)~=4) )
   error('Screws:RigidOrientation:Input','Input matrix is not a 4x4 matrix.\n');
end
%if( abs(det(g(1:3,1:3))) < 0.9 | abs(det(g(1:3,1:3))) > 1.1 )
%   error('Screws:RigidOrientation:Input','Input matrix''s rotation matrix is not determinant 1.\n');
%end

Rotate = [ g(1,1), g(1,2), g(1,3) ;
           g(2,1), g(2,2), g(2,3) ;
           g(3,1), g(3,2), g(3,3) ];

return;
