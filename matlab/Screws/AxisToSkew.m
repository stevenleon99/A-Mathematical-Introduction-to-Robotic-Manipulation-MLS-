function Skew = AxisToSkew(uvec)
% Skew = AxisToSkew(uvec)
% Calculates a skew symetric matrix for a unit vector.
% That is, given a 3x1 unit direction vector uvec, this returns 
% a 3x3 skew-symetric matrix Skew.  

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
% Check input
%
error(nargchk(1, 1, nargin))
if( (size(uvec,1)~=3) | (size(uvec,2)~=1) )
   error('Screws:AxisToSkew:Input','Input vector is not a 3x1 vector.\n');
end
if(norm(uvec) > 1.001 | norm(uvec) < 0.999) 
   error('Screws:AxisToSkew:Input','Input vector is not a unit vector with norm=1.\n');
end

%
% Compute the Skew Symetric Matrix of the unit vector
%
uvec=uvec';
Skew = [ 0      , -uvec(3),  uvec(2)   ;
        uvec(3),  0      , -uvec(1)   ;
       -uvec(2),  uvec(1),  0        ];

  return;
