function uvec = RotationAxis(R)
% uvec = RotationAxis(R)
% Calculates the rotation axis of a rotation matrix.
% That is, given a 3x3 rotation matrix, this returns the 3x1 axis of rotation.  

%
%  Written by Dr. Kevin Otto
%  See LICENSE.txt for copyright info.
%

%
% Check inputs
%
error(nargchk(1, 1, nargin))
if( (size(R,1)~=3) | (size(R,2)~=3) )
   error('Screws:RotationAxis:Input','Input matrix is not 3x3.\n');
end
%if(det(R) > 1.001 | det(R) < 0.999) 
%   error('Screws:RotationAxis:Input','Input matrix does not have det=1.\n');
%end
if(R(1,1) == 1 & R(2,2) == 1 & R(3,3) == 1)
   error('Screws:RotationAxis:Input','Input matrix is the identity matrix and so no unique solution.\n');
end

%
% Solve for the angle and vector
%
% uvec = 1/2/sin(acos((trace(R)-1)/2)) * [R(3,2)-R(2,3); R(1,3)-R(3,1); R(2,1)-R(1,2)];  
uvec = [R(3,2)-R(2,3); R(1,3)-R(3,1); R(2,1)-R(1,2)];
uvec = uvec/norm(uvec);
 
return;
