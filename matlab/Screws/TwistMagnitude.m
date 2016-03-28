function Magnitude = TwistMagnitude(xi)
% Magntiude = TwistMagnitude(xi)
% Returns the magnitude of a 6x1 twist.

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
   error('Screws:ScrewToTwist:Input','Input vector is not a 6x1 vector.\n');
end

v = xi(1:3);
w = xi(4:6);

if( w == [0,0,0] )
    M = norm(v);
else
    M = norm(w);
end

Magnitude = M;

return;
