function Twist = ScrewToTwist(h,q,w)
% Twist = ScrewToTwist(h,q,w)
% Return the twist coordinates of a screw with pitch h through the point q
% and in the direction w. If h == Infinity, then a pure translational twist 
% is generated. In this case, q is ignored and w gives the direction of translation.

%
%  Written by Dr. Kevin Otto
%  See LICENSE.txt for copyright info.
%
%
%  Robust Systems and Strategy LLC
%  otto@robuststrategy.com
%  Version 1.0
%

error(nargchk(3, 3, nargin))
if( (size(h,1)~=1) | (size(h,2)~=1) )
   error('Screws:ScrewToTwist:Input','Input pitch h is not a scalar.\n');
end
if( (size(w,1)~=3) | (size(w,2)~=1) )
   error('Screws:ScrewToTwist:Input','Input direction w is not a 3x1 vector.\n');
end
if( (size(q,2)~=1) | ~((size(q,1)==3)|(size(q,1)==4)) )
   error('Screws:ScrewToTwist:Input','Input point q is not a 3x1 or 4x1 vector.\n');
end
if( (norm(w) < 0.9) | (norm(w) > 1.1) )
   error('Screws:ScrewToTwist:Input','Input direction w is not a unit vector.\n');
end

if(h==Inf) 
  v = w(1:3);
  ww = [0;0;0];
else
  v = -AxisToSkew(w(1:3))*q + h*w(1:3);
  ww = w(1:3);
end

Twist = [ v(1), v(2), v(3), ww(1), ww(2), ww(3) ];

return;
