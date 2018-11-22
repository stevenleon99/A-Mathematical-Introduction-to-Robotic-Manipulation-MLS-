# lyapunov.cmd - SST file to generate lyapunov level sets
# RMM, 4 Aug 93

range obs[1-110]

set t = (obsno-1)/100;
calc eps = 0.1

calc a1 = 3.89
set x1 = a1*cos(2*PI*t)
set y1 = a1*sin(2*PI*t)

set sx1 = x1;
set sy1 = -eps*x1 + (t < 0.5 ? 1 : -1) * sqrt(a1^2 - x1^2 + eps^2 * x1^2);

calc a2 = 8.53
set x2 = a2*cos(2*PI*t)
set y2 = a2*sin(2*PI*t)

set sx2 = x2;
set sy2 = -eps*x2 + sqrt(a2^2 - x2^2 + eps^2 * x2^2);
set sy2 = -sy2[-50]; obs[51-101]

write file[round.dat] var[t, x1 y1, x2 y2]
write file[skew.dat] var[t, sx1 sy1, sx2 sy2]
