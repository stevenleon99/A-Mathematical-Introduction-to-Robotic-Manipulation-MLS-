function Adjoint = RigidAdjoint(g)
% Inverse = RigidAdjoint(g)
% Computes the 6x6 adjoint matrix of the screw corresponding to a 4x4 homogenous transform.  

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
if( det(g(1:3,1:3)) < 0.9 | det(g(1:3,1:3)) > 1.1 )
   error('Screws:RigidOrientation:Input','Input matrix''s rotation matrix is not determinant 1.\n');
end

R = RigidOrientation(g);
p = RigidPosition(g);
if( norm(p(1:3))==0 )
    u = [0;0;1];
else
    u = p(1:3)/norm(p(1:3));
end

Adjoint = [        R, AxisToSkew(u)*R ;
            zeros(3), R               ];

return;
