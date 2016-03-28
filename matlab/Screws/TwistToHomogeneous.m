function HTM = TwistToHomogeneous(xi)
% HTM = TwistToHomogeneous(xi)
% Calculates the 4x4 homogenous transform HTM from a 6x1 twist vector.

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
if( (size(xi,1)~=1) | (size(xi,2)~=6) )
   error('Screws:TwistToHomogeneous:Input','Input vector is not a 6x1 vector.\n');
end

R = AxisToSkew(xi(4:6)');
p = xi(1:3);
HTM = [ R(1,1), R(1,2), R(1,3), p(1) ;
        R(2,1), R(2,2), R(2,3), p(2) ;
        R(3,1), R(3,2), R(3,3), p(3) ;
        0,      0,      0,      1    ];

return;
