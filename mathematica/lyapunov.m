(*
 * lyapunov.m - generate plots for lyapunov section
 *
 * RMM, 29 Dec 92
 *
 *)

<<Simulate.m
Needs["Graphics`ImplicitPlot`"];

(* System description *)
BuildSystem[osc,
  States[{x1, x2}];
  Derivs[{dx1, dx2}];
  Params[{M = 1, K = 1, B = 0.1}];

  dx1 = x2;
  dx2 = (-K x1 - B x2) / M;
];

(* Define some different lyapunov functions *)
V1 = 1/2 x1^2 + x2^2

(* Run a simulation to get a phase portrait *)
Simu[osc, {0, 50, 0.1}, Initial->{10,0}, Output->"harmonic.dat"]

(*****
(* Read the data back into mathematica *)
{time, x1data, x2data} =
  Transpose[ReadList["store.d", {Number, Number, Number}]];

graph = Show[
  ListPlot[Transpose[{x1data, x2data}], PlotJoined->True],
  ContourPlot[V1, {x1, -10, 10}, {x2, -10, 10},
    ContourShading->False, Contours->{50, 40, 30, 20, 10},
    ContourStyle->Dashing{0.1,0.1}],
  AspectRatio->0.75
]

Display["ch4-lyapunov.mps", graph]
*****)
