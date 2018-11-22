(*
 * scaragrasp.m - grasping example
 *
 * Richard M. Murray
 * 27 Jan 92
 *
 * This uses the notation from ME230, *not* the notation in the book.
 *)

<<scara.m			(* read scara kinematics *)
<<matlib.m

(* Construct the change of coordinates from contact to spatial coordinates *)
(* Only do the construction at the identity object configuration *)
Rpo = IdentityMatrix[3];  ppo = {0,0,a};
Rc1s1 = {{0,0,1}, {1,0,0}, {0,1,0}};  pc1s1 = {-a,0,-b};
B = {{1,0,0,0}, {0,1,0,0}, {0,0,1,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,1}};

gc1s1 = RPToHomogeneous[Rc1s1, pc1s1];

J11 = Transpose[B] . RigidAdjoint[gc1s1] . Js;

