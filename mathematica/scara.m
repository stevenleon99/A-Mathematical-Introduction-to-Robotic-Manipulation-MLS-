(*
 * scara.m - Kinematics for SCARA robot
 *
 * Richard M. Murray
 * 22 January 1992
 *
 * This file has the solutions for the following examples:
 *
 *	exmp 3-1:	SCARA forward kinematics
 *	exmp 3-8:	Jacobian for a SCARA robot
 *	exmp 4-4:	Dynamics of an idealized SCARA ma 
 *)

<<RobotLinks.m

q = {th1,th2,th3,th4}
w = {dth1,dth2,dth3,dth4}

(* twist axes for SCARA robot, reference frame at base *)
xi1 = {0,0,0, 0,0,1};		(* base *)
xi2 = {l1,0,0, 0,0,1};		(* elbow *)
xi3 = {l1+l2,0,0, 0,0,1};	(* wrist revolute *)
xi4 = {0,0,1, 0,0,0};		(* wrist prismatic *)
xi0 = {0,0,0, 0,0,0};		(* dummy twist *)

gst0 = RPToHomogeneous[IdentityMatrix[3], {0,l1+l2,0}];

(* Calculate out the individual exponentials *)
expxi1 = TwistExp[xi1,th1];
expxi2 = TwistExp[xi2,th2];
expxi3 = TwistExp[xi3,th3];
expxi4 = TwistExp[xi4,th4];

(* Forward Kinematics *)
gst = Simplify[ expxi1 . expxi2 . expxi3 . expxi4 . gst0 ];

(* Spatial Jacobian *)
Js = Simplify[StackCols[
  xi1,
  RigidAdjoint[TwistExp[xi1, th1]] . xi2,
  RigidAdjoint[TwistExp[xi1, th1] . TwistExp[xi2, th2]] . xi3,
  RigidAdjoint[
    TwistExp[xi1, th1] . TwistExp[xi2, th2] . TwistExp[xi3, th3]
  ] . xi4
]];

(*
 * Dynamics
 *
 * This is used for the example in Chapter 4, after the product of
 * exponentials description of the dynamics.  The dynamics are derived
 * in *two* ways.  First using the standard approach (with the body
 * Jacobian) and then using the adjoint matrices.  The answers *should*
 * be the same.
 *
 *)

(* Define the link frames *)
g10 = RPToHomogeneous[IdentityMatrix[3], {0,r1,l0}];
g20 = RPToHomogeneous[IdentityMatrix[3], {0,l1+r2,l0+h2}];
g30 = RPToHomogeneous[IdentityMatrix[3], {0,l1+l2,l0+h3}];
g40 = RPToHomogeneous[IdentityMatrix[3], {0,l1+l2,l0+h3+h4}];

(* Compute out the body Jacobians *)
J1 = Simplify[BodyJacobian[{xi1,th1}, {xi0,th2}, {xi0,th3}, {xi0,th4}, g10]];
J2 = Simplify[BodyJacobian[{xi1,th1}, {xi2,th2}, {xi0,th3}, {xi0,th4}, g20]];
J3 = Simplify[BodyJacobian[{xi1,th1}, {xi2,th2}, {xi3,th3}, {xi0,th4}, g30]];
J4 = Simplify[BodyJacobian[{xi1,th1}, {xi2,th2}, {xi3,th3}, {xi4,th4}, g40]];

M1 = DiagonalMatrix[{m1,m1,m1,Ix1,Iy1,Iz1}];
M2 = DiagonalMatrix[{m2,m2,m2,Ix2,Iy2,Iz2}];
M3 = DiagonalMatrix[{m3,m3,m3,Ix3,Iy3,Iz3}];
M4 = DiagonalMatrix[{m4,m4,m4,Ix4,Iy4,Iz4}];

Inertia = Simplify[
  Transpose[J1].M1.J1 + Transpose[J2].M2.J2 + 
  Transpose[J3].M3.J3 + Transpose[J4].M4.J4
]

Coriolis = InertiaToCoriolis[Inertia, q, w];
gamma[i_,j_,k_] := D[Coriolis, w[[k]]][[i,j]]

For[i=1,i<=4,++i,
  For[j=1,j<=4,++j,
    For[k=1,k<=4,++k,
      Print["gamma ", i,j,k, " = ", Simplify[gamma[i,j,k]]]]]]
