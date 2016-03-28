function Inverse = RigidInverse(g)
% Inverse = RigidInverse(g)
% Computes the inverse of a 4x4 homogenous transform.  

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
   error('Screws:RigidINverse:Input','Input matrix is not a 4x4 matrix.\n');
end
if( det(g(1:3,1:3)) < 0.9 | det(g(1:3,1:3)) > 1.1 )
   error('Screws:RigidInverse:Input','Input matrix''s rotation matrix is not determinant 1.\n');
end

R=g(1:3,1:3);
p=g(1:3,4);

p=-R'*p;
R=R';
Inverse = [ R(1,1), R(1,2), R(1,3), p(1) ;
            R(2,1), R(2,2), R(2,3), p(2) ;
            R(3,1), R(3,2), R(3,3), p(3) ;
                 0,      0,      0,    1 ];

return;
