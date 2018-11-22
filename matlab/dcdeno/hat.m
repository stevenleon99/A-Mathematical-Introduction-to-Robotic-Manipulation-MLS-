function hat_w = hat(w)
% hat, returns 3x3 skew-symmetric matrix from vector
% hat(w) = w^
%
% DC Deno 9-26-91

hat_w = [0     -w(3)   w(2);
         w(3)   0     -w(1);
        -w(2)   w(1)   0];
