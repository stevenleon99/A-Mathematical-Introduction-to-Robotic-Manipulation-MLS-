function Axis = TwistAxis(xi)
% Axis = TwistAxis(xi)
% Returns the axis of a 6x1 twist.

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
   error('Screws:TwistAxis:Input','Input vector is not a 6x1 vector.\n');
end
if( norm(xi)==0 )
   error('Screws:TwistAxis:Input','Input vector is a zero vector.\n');
end

v = xi(1:3);  % Note this is a row vector
w = xi(4:6);  % Note this is a row vector

if ( w == [0,0,0] ) 
    w = v / sqrt(v*v');
    v = [0;0;0];
else
    v = AxisToSkew(w') * v' / (w*w');
    w = w / (w*w');
end

Axis = [ v', w ];

return;
