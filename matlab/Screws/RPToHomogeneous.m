function TForm = RPToHomogeneous(R,p)
% HTM = RPToHomogeneous(R,p)
% Calculates the 4x4 homogenous transform HTM of a 3x3 rotation matrix R
% and a 3x1 displacement vector p.

%
%  Written by Dr. Kevin Otto
%  See LICENSE.txt for copyright info.
%
%
%  Robust Systems and Strategy LLC
%  otto@robuststrategy.com
%  Version 1.0
%

error(nargchk(2, 2, nargin))

if( (size(p,1)~=3) | (size(p,2)~=1) )
 if( (size(p,1)~=4) )
   error('Screws:RPToHomogeneous:Input','Input p vector is not a 3x1 or 4x1 vector.\n');
 end
end

if( size(R,1)~=3 ) 
 error('Screws:RPToHomogeneous:Input','Input R is not a 3 row matrix or vector.\n');
else
 if( (size(R,2)~=3) && (size(R,2)~=1) )
   error('Screws:RPToHomogeneous:Input','Input R is not a 3x3 matrix or 3x1 vector.\n');
 else
  if (size(R,2) == 1)
    RR = AxisToSkew(R);
  else
    RR = R;
  end
 end
end

p=p';
TForm = [ RR(1,1), RR(1,2), RR(1,3), p(1)  ;
          RR(2,1), RR(2,2), RR(2,3), p(2)  ;
          RR(3,1), RR(3,2), RR(3,3), p(3)  ;
               0,      0,      0,    1 ];

return;
