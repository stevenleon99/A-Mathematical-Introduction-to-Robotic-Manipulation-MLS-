function uvec = SkewToAxis(Skew)
% Skew = AxisToSkew(uvec)
% Calculates a 3x1 unit direction vector uvec for skew symetric matrix.
% That is, given a 3x3 skew symetric matrix Skew, this returns 
% a 3x1 unit vector of the rotation.  

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
error(nargchk(1, 1, nargin))
if( (size(Skew,1)~=3) | (size(Skew,2)~=3) )
   error('Screws:SkewToAxis:Input','Input matrix is not a 3x3 matrix.\n');
end

%
% Compute the the unit vector of the Skew Symetric Matrix
%
   uvec(1) = Skew(3,2);
   uvec(2) = Skew(1,3);
   uvec(3) = Skew(2,1);

   if(norm(uvec)==0)
       uvec = uvec'
   else
       uvec=uvec'/norm(uvec);
   end
       
return;
