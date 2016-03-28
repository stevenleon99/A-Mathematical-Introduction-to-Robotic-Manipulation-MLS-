function Position = RigidPosition(g)
% Position = RigidPosition(g)
% Extracts the 3x1 position vector from a 4x4 homogenous transform.  

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

Position = [ g(1,4) ;
             g(2,4) ;
             g(3,4) ];

return;
