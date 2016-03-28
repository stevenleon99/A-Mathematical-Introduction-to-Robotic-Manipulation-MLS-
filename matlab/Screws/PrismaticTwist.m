function Twist = PrismaticTwist(q,w)
% Twist = PrismaticTwist(q,w)
% Return the twist coordinates of a screw with a unit direction vector w through the point q. 
% This is 6x1 twist vector for a prismatic joint in the direction w through the point q.  

%
%  Written by Dr. Kevin Otto
%  See LICENSE.txt for copyright info.
%
%
%  Robust Systems and Strategy LLC
%  otto@robuststrategy.com
%  Version 1.0
%

error(nargchk(2, 2, nargin))    % there should be 2 inputs
if( (size(w,1)~=3) | (size(w,2)~=1) )
   error('Screws:ScrewToTwist:Input','Input rotation vector w is not a 3x1 vector.\n');
end
if( (size(q,2)~=1) | ~((size(q,1)==3)|(size(q,1)==4)) )
   error('Screws:ScrewToTwist:Input','Input point q is not a 3x1 or 4x1 vector.\n');
end
if( (norm(w) < 0.9) | (norm(w) > 1.1) )
   error('Screws:ScrewToTwist:Input','Input direction w is not a unit vector.\n');
end

Twist = [ w(1), w(2), w(3), 0, 0, 0 ];

return;
