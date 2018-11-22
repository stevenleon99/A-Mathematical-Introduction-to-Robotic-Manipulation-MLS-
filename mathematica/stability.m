(*
 * stabliblity.m - generate simulation data for stability plots
 *
 * RMM, 4 Aug 93
 *
 * The system we simulate is an damped pendulum.  By varying the damping
 * coefficient and the equilibrium point, we are able to get examples
 * of isL stable, asy stable, and saddle behavior.
 *
 * This file uses Simulate.m, a simulation program for Mathematica.
 * Simulate.m is an undocumented, unsupported package written by
 * John Hauser and Richard Murray.  Contact oen of them if you want to
 * try getting it to work on your system.
 *)

<<Simulate.m

(* Pendulum *)
BuildSystem[invpnd,
  States[{th,omega}];
  Derivs[{dth, domega}];
  Params[{m=1,g=9.8,l=4,b=0,off=0}];

  dth = omega;
  domega = 1/(m l^2) (-m g l Sin[th + off] - b dth);
];

(* Stable (isL) equilibrium point *)
Parm[invpnd, b->0, off->0];
Simu[invpnd,{0,10}, Initial->{2.9,0}, Output->"stable.d1", Method->lsoda];
Simu[invpnd,{0,10}, Initial->{2,0}, Output->"stable.d2", Method->lsoda];
Simu[invpnd,{0,10}, Initial->{1,0}, Output->"stable.d3", Method->lsoda];
Simu[invpnd,{0,10}, Initial->{0.5,0}, Output->"stable.d4", Method->lsoda];
Simu[invpnd,{0,10}, Initial->{0.2,0}, Output->"stable.d5", Method->lsoda];

(* Asymptotically stable eq pt *)
Parm[invpnd, b->10, off->0];
Simu[invpnd,{0,10}, Initial->{2.9,0}, Output->"asystable.d1", Method->lsoda];
Simu[invpnd,{0,10}, Initial->{-2,0}, Output->"asystable.d2", Method->lsoda];
Simu[invpnd,{0,10}, Initial->{1,0}, Output->"asystable.d3", Method->lsoda];
Simu[invpnd,{0,10}, Initial->{2,0}, Output->"asystable.d4", Method->lsoda];
Simu[invpnd,{0,10}, Initial->{-2.9,0}, Output->"asystable.d5", Method->lsoda];

(* Saddle point *)
Parm[invpnd, b->10, off->Pi];
Simu[invpnd,{0,10}, Initial->{0.25,-0.2}, Output->"saddle.d1", Method->lsoda];
Simu[invpnd,{0,10}, Initial->{-0.25,0.2}, Output->"saddle.d2", Method->lsoda];
Simu[invpnd,{0,10}, Initial->{-0.10,0.25}, Output->"saddle.d3", Method->lsoda];
Simu[invpnd,{0,10}, Initial->{0.10,-0.25}, Output->"saddle.d4", Method->lsoda];
Simu[invpnd,{0,10}, Initial->{-0.15,0.25}, Output->"saddle.d5", Method->lsoda];
Simu[invpnd,{0,10}, Initial->{0.15,-0.25}, Output->"saddle.d6", Method->lsoda];
