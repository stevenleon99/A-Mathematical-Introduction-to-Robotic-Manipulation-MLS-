(*
 * redundant.m - analysis of three-link planar redundant manipulator
 *
 * RMM, 21 Sep 92
 *
 * This example is used in Chapter 3 (kinematics) and again later in
 * the book (location not set yet).
 * 
 *)

<<Jac.m

mod[x_] := N[x - Floor[(x+Pi)/(2Pi)] * 2Pi];
mod1[x_] := N[x - Floor[x/(2Pi)] * 2Pi];

(* Function to solve the inverse kinematics of a two link manipulator *)
inverse[x_, y_, flip_] :=
  Module[
    {th1, th2, alpha, r = Sqrt[x^2 + y^2]},
    alpha = ArcCos[(2 - r^2)/2];    th2 = Pi + flip alpha;
    beta = ArcCos[r/2];		    th1 = ArcTan[y/x] + flip beta;
    {mod[N[th1]], mod[N[th2]]}
  ];

(* Now swing the last joint around and solve for remaining variables *)
genpt[th3_] :=
  Join[
    inverse[1 + Sin[th3], 1 + Cos[th3], If[N[th3] > N[2Pi], -1, 1]],
    {th3}]

ParametricPlot3D[{genpt[th][[1]], genpt[th][[2]], mod1[th]}, {th, 0, 4Pi}, PlotPoints->100]
