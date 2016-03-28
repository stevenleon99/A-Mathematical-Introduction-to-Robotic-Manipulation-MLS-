function Twist = RigidTwist(g)
% Twist = RigidTwist(g)
% Calculates the 6x1 twist vector that generated the 4x4 homogenous transform g 
% USes RotationAxis to find the twist direction w

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
   error('Screws:RigidTwist:Input','Input matrix is not a 4x4 vector.\n');
end

R = RigidOrientation(g); 
p = RigidPosition(g);
w = RotationAxis(R);
theta = RotationAngle(R);

if(theta==0) 
      theta = norm(p);
      v = p/norm(p);
      w = [0;0;0];
else
%      (* Solve a linear equation to figure out what v is *)   
      v = inv((eye(3) - R)*AxisToSkew(w)+theta*(w'*w)*eye(3))*p;
end

w=w'; v=v';
Twist = [ v(1), v(2), v(3), w(1), w(2), w(3) ];

return;
