(*
 * stanford.m - Kinematics for Stanford arm 
 *
 * Richard M. Murray
 * 22 January 1992
 *
 *)

<<RobotLinks.m

(* twist axes for arm robot, reference frame at base *)
xi1 = RevoluteTwist[{0,0,l0}, {0,0,1}];		(* base *)
xi2 = RevoluteTwist[{0,0,l0}, {-1,0,0}];
xi3 = PrismaticTwist[{0,l1,l0}, {0,1,0}];	(* "elbow" *)
xi4 = RevoluteTwist[{0,l1+l2,l0}, {0,0,1}];	(* wrist *)
xi5 = RevoluteTwist[{0,l1+l2,l0}, {-1,0,0}];
xi6 = RevoluteTwist[{0,l1+l2,l0}, {0,1,0}];

g0 = RPToHomogeneous[IdentityMatrix[3], {0,l1+l2,l0}];

(* Forward Kinematics *)
g = Simplify[
  TwistExp[xi1,th1] . TwistExp[xi2,th2] .
  TwistExp[xi3,th3] . TwistExp[xi4,th4] .
  TwistExp[xi5,th5] . TwistExp[xi6,th6] . g0,
  Trig->False
];

(* Spatial Jacobian *)
Js = Simplify[
  RobotSpatialJacobian[
    {xi1, th1}, {xi2, th2}, {xi3, th3},
    {xi4, th4}, {xi5, th5}, {xi6, th6}
  ], Trig->False
];

