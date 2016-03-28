function Vector = VectorToHomogeneous(pvec)
% Vector = VectorToHomogeneous(pvec)
% Calculates the 4x1 homogenous version of a 3x1 vector pvec.
% Simply put, padds the vector with an addition '0'.  
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
if( (size(pvec,1)~=3) | (size(pvec,2)~=1) )
   error('Screws:PointToHomogeneous:Input','Input vector is not a 3x1 vector.\n');
end

pvec = pvec';
Vector = [pvec(1); pvec(2); pvec(3); 0];

return;
