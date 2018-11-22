(*
 * Example 6.4:  Tendon kinematics for two-link tendon-driven finger
 *
 * RMM, 29 Dec 94
 *
 *)

<<Jac.m

(* Extension functions *)
h2 = l2 - R1 th1;
h3 = l3 + R1 th1;
h1 = l1 + 2 Sqrt[a^2 + b^2] Cos[ArcTan[a/b] + th1/2] - 2 b - R2 th2;
h4 = l4 + R1 th1 + R2 th2;
h = {h1, h2, h3, h4}

(* Compute the coupling matrix *)
P = Transpose[Jac[h, {th1,th2}]]
