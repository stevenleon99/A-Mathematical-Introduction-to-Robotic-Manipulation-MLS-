function Rotate = SkewExp(uvec, theta)
% R = SkewExp(uvec, theta)
% Calculates the matrix exponential for a skew symmetric matrix.
% That is, given a 3x1 unit direction vector uvec and amount of rotation
% theta about that axis, this returns the 3x3 rotation matrix R.  

%
%  Written by Dr. Kevin Otto
%  See LICENSE.txt for copyright info.
%
%
%  Robust Systems and Strategy LLC
%  otto@robuststrategy.com
%  Version 1.0
%


%
% Check inputs
%
error(nargchk(2, 2, nargin))
if( (size(uvec,1)~=3) | (size(uvec,2)~=1) )
   error('Screws:SkewExp:Input','Input vector is not a 3x1 vector.\n');
end
if(norm(uvec) > 1.001 | norm(uvec) < 0.999) 
   error('Screws:SkewExp:Input','Input vector is not a unit vector with norm=1.\n');
end
if((size(theta,1)~=1) | (size(theta,2)~=1) )
   error('Screws:SkewExp:Input','Input theta is not a scalar.\n');
end

%
% Compute the Skew Symetric Matrix of the unit vector
%

uvec=uvec';
R = [ 0      , -uvec(3),  uvec(2)   ;
      uvec(3),  0      , -uvec(1)   ;
      -uvec(2),  uvec(1),  0        ];

%
% Now Use Rodrigues's formula
%
  Rotate =  eye(3) + sin(theta)*R + (1 - cos(theta))*R*R;

  return;
