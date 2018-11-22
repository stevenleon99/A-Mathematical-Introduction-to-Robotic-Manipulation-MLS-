Off[General::spell1];
BeginPackage["RobotLinks`"];

RotationQ::usage="RotationQ[m] tests whether matrix m is a rotation  
matrix";

RPToHomogeneous::usage="RPToHomogeneous[R,p] forms homogeneous matrix from  
rotation matrix R
 and position vector p";

RotationAxis::usage="RotationAxis[R] finds the rotation axis given  
the rotation matrix R";

Skew::usage="Skew[w] generates skew symmetric matrix given 3 vector  
w";

UnSkew::usage="UnSkew[S] extracts vector w from skewsymmetric matrix  
S";

SkewExp::usage=
 "SkewExp[w,(Theta)] gives the matrix exponential of an axis w.
  Default value of Theta is 1.";

TwistExp::usage=
 "TwistExp[xi,(Theta)] gives the matrix exponential of a twist xi.
  Default value of Theta is 1.";
	
TwistToHomogeneous::usage="TwistToHomogeneous[xi] converts xi from a 6 vector to a  
4X4 matrix";

RigidOrientation::usage="RigidOrientation[g] extracts rotation matrix R from  
g";

RigidPosition::usage="RigidPosition[g] extracts position vector p from g";

RigidTwist::usage="RigidTwist[g] extracts 6 vector xi from g";

TwistPitch::usage="TwistPitch[xi] gives pitch of screw corresponding to xi  
6vector or xi matrix";

TwistAxis::usage="TwistAxis[xi] gives axis of screw corresponding to xi 6vector  
or xi matrix";

TwistMagnitude::usage="TwistMagnitude[xi] gives magnitude of screw  
corresponding to xi 6vector or xi matrix";

RigidAdjoint::usage=
  "RigidAdjoint[g] gives the adjoint matrix corresponding to g";

PointToHomogeneous::usage=
  "PointToHomogeneous[q] gives the homogeneous representation of a point";

VectorToHomogeneous::usage=
  "VectorToHomogeneous[q] gives the homogeneous representation of a vector";
  
CrossProduct::usage="CrossProduct[Vector1,Vector2] gives the cross product of two 3 D vectors";

RevoluteTwist::usage="RevoluteTwist[q,w] gives the Xi 6-vector corresponding to point q on the axis and a screw with axis w for a revolute joint";

PrismaticTwist::usage="PrismaticTwist[q,w] gives the Xi 6-vector corresponding to point q on the axis and a screw with axis w for a prismatic joint";


WrenchTransformation::usage="
  WrenchTransformation[p,R] constructs the wrench transformation matrix given p and R
WrenchTransformation[g] gives the wrench transformation matrix given g";

ContactBasis::usage="Basis[PCNF/PCWF/SF]returns the corresponding basis matrix ";

Magnitude::usage="Magnitude[w] returns the magnitude of a 3 vector w";


RobotSpatialJacobian::usage="RobotSpatialJacobian[{Xi1,Theta1},{Xi2,Theta2},..] gives the jacobian of a robot with the given Xi and Theta  (in proper sequence)";

StackRows::usage="StackRows[Mat1,Mat2,..] stacks matrix rows together";

StackCols::usage="StackCols[Mat1,Mat2,...] stacks matrix columns together";

Begin["`Private`"];
(*ERROR MESSAGES*)

Skew::wrongD="`1` is not a 3 vector.";
UnSkew::notskewsymmetric ="`1` is not a skew symmetric matrix";
gen::wrongD="`1`Dimensions of input matrices incorrect.";

(*MODIFICATIONS TO EXISTING DEFS*)

(
Unprotect[ArcCos];
ArcCos[Cos[expr_]]:=expr;
Protect[ArcCos];
)
(
Unprotect[Times];
Times[1.,expr_]:=expr;
Protect[Times];
)

(*TESTING FUNCTIONS*)

RotationQ[mat_]:=    (*Tests whether rotation matrix*)
	Block[{mag,prod,cond,invmat},
		Which[
		NumericQ[mat] && Dimensions[mat]=={3,3},
		mag=Det[mat];
		prod=mat.Inverse[mat];
		cond=True;
		For[i=1,i<=3 && cond==True,i++,
			For[j=1,j<=3 && cond==True,j++,
				If [i!=j,cond=cond && TrueQ[Abs[0.-prod[[i,j]]]<=(10.^-4)],
					 cond=cond && TrueQ[Abs[1.-prod[[i,j]]]<=(10.^-4)]]
						 ]];
		TrueQ[Abs[1-mag]<=(10.^-4) && cond],
		Not[NumericQ[mat]]  && Dimensions[mat]=={3,3},
		prod=Simplify[mat.Transpose[mat]];
		mag=Det[mat];
		If[NumericQ[prod] && NumericQ[mag],
		cond=TrueQ[prod==IdentityMatrix[3] && mag==1],
		cond="Null"]
		]
		
		];


so3Q[mat_]:=       (*Tests whether skew symmetric*)
	Block[{w1,w2,w3},
		If[Dimensions[mat]=={3,3},
TrueQ[mat[[1,1]]==mat[[2,2]]==mat[[3,3]]==0 && mat[[1,2]]==-mat[[2,1]] && mat[[1,3]]==-mat[[3,1]] && mat[[2,3]]==-mat[[3,2]]],
		Message[gen::wrongD,"  so3Q :  "]]];
		
OmegaQ[mat_]:=  TrueQ[Dimensions[mat]=={3} || (Dimensions[mat]=={3,3}  
&& so3Q[mat])];
 (*Tests whether w or what*)
 

XiQ[mat_]:=	TrueQ[Dimensions[mat]=={6} || Dimensions[mat]=={4,4}];
	
 (*Tests whether xi or xihat*)		

NumericQ[w_]:=  (*Tests whether w has any symbolic elements*)
	Block[{t1,t2,t3,thing,cond,i,dim},
		t1=NumberQ[w];
		t2=VectorQ[w];
		t3=MatrixQ[w];
		If[t1||t2||t3,
		If[t1,True,
		Which[t3,thing=Flatten[w],t2,thing=w];
		dim=Dimensions[thing];
		cond=True;
		For[i=1,i<=dim[[1]] && cond==True,i++,cond=cond && NumberQ[thing[[i]]]];
		cond],
		False]];

(*INTERNAL FUNCTIONS*)


Versine[t_]:=1-Cos[t];

Magnitude[w_]:=(Transpose[w] . w)^(1/2);

CrossProduct[a_,b_]:=
			Block[{v},
				v={a[[2]]b[[3]]-b[[2]]a[[3]],
				   a[[3]]b[[1]]-a[[1]]b[[3]],
				   a[[1]]b[[2]]-a[[2]]b[[1]]}];
				        


XiToW[xi_]:=  (*Extracts w from xi 6 or xi 4X4 *)
	Block[{dim,omega},
		dim=Dimensions[xi];
		Which[dim=={6},
		omega=Take[xi,-3],
		dim=={4,4},
		omega={xi[[3,2]] , xi[[1,3]] , xi[[2,1]]}]];
		
XiToV[xi_]:=  (*Extracts v from xi 6 or xi 4X4 *)
	Block[{dim,vee},
		dim=Dimensions[xi];
		Which[dim=={6},
		vee=Take[xi,3],
		dim=={4,4},
		vee={xi[[1,4]],xi[[2,4]],xi[[3,4]]}]];

				  

(*PACKAGE FUNCTIONS*)				      

(* Convert a point to homogeneous coordinates *)
PointToHomogeneous[p_] := Append[p, 1] /; VectorQ[p] && Length[p] == 3;
	
RPToHomogeneous[r_,p_]:= (* Makes homogeneous r,p*)
	Block[{hmgmat},
		If[Dimensions[r]=={3,3} && Dimensions[p]=={3},
		hmgmat={Join[r[[1]],{p[[1]]}],
		   		   Join[r[[2]],{p[[2]]}],
		   		   Join[r[[3]],{p[[3]]}],	
		   		   {0,0,0,1}},
		    Message[gen::wrongD," RPToHomogeneous :     "]]];
		    
		    
PointToHomogeneous[q_]:=      (*Homogeneous representation of a point*)
	Block[{pthomog},
		If[Dimensions[q]=={3},
		pthomog={q[[1]],q[[2]],q[[3]],1},
		Message[gen::wrongD," PointToHomogeneous :     "]]];
		    

VectorToHomogeneous[q_]:=      (*Homogeneous representation of a vector*)
	Block[{pthomog},
		If[Dimensions[q]=={3},
		pthomog={q[[1]],q[[2]],q[[3]],0},
		Message[gen::wrongD," VectorToHomogeneous :     "]]];
	
	
RotationAxis[r_]:=(*Finds rotation axis*)
	Block[ {r1,r2,r3,rx,p,trace,theta,arg},
		If[RotationQ[r] || Not[NumericQ[r]],
		trace=r[[1,1]]+r[[2,2]]+r[[3,3]];
		arg=(trace-1)/2;
		theta=ArcCos[arg];
		cond=TrueQ[NumericQ[r] && theta==0];
		If[cond,
		Print ["No rotation - any arbitrary axis will do"];
		rx={0,0,0},
		r1=r[[3,2]]-r[[2,3]];
		r2=r[[1,3]]-r[[3,1]];
		r3=r[[2,1]]-r[[1,2]];
		rx={r1,r2,r3};
		p=((Sin[theta])^(-1)/2) * rx],
		Print[r," is not a Rotation Matrix"]]];	
						
Skew[omega_]:=(*Generates skew symmetric matrices*)
	Block[
		{w1,w2,w3,skwmat},
		If[Dimensions[omega]=={3},
		w1=omega[[1]];
		w2=omega[[2]];
		w3=omega[[3]];
		skwmat={{0,-w3,w2},
		    {w3,0,-w1},
		    {-w2,w1,0}},
		   Message[Skew::wrongD,omega]]];

UnSkew[what_]:=(*Extracts w from skew symmetric matrix w*)
	Block[{w},
		If[so3Q[what],
		w={what[[3,2]] , what[[1,3]] , what[[2,1]]},
		Message[UnSkew::not_skew_symmetric,omega]]];


SkewExp[w_,theta_:1]:=(*Matrix exponential for w matrices*)
	Block[{dim,matexp,prop},
		dim=Dimensions[w];
		If[dim=={3},prop=Skew[w],prop=w];
		matexp=IdentityMatrix[3]+Sin[theta] prop+Versine[theta] MatrixPower[prop,2]
		]/;OmegaQ[w]
		
		
RevoluteTwist[q_,w_]:=      (*Gives Xi 6 vector given axis and point on axis*)

	Block[{xi},
		xi=Flatten[{CrossProduct[q,w],w}]];
		
PrismaticTwist[q_,w_]:=(*Gives Xi 6 vector given point on axis q and axis w*)
	Block[{xi},
		xi=Flatten[{w,{0,0,0}}]];
		

TwistExp[xi_,theta_:1]:=(*Converts  xi to  matrix*)
	Block[{w,v,wmat,vmat,r,cond},
		w=XiToW[xi];
		v=XiToV[xi];
		cond=TrueQ[w!={0,0,0} || Not[NumericQ[w]]];
		If[cond,
		wmat=SkewExp[w,theta];
		vmat=(IdentityMatrix[3]-wmat).CrossProduct[w,v]+w(w.v)theta,
		wmat=IdentityMatrix[3];
		vmat=v*theta];
		r=RPToHomogeneous[wmat,vmat]
		]/;XiQ[xi]	
	
		
TwistToHomogeneous[xi_]:=(*Converts xi 6 to xi 4X4*)
	Block[{wzet,hwzet,vzet,dum},
		If [Dimensions[xi]=={6},
		wzet=XiToW[xi];
		vzet=XiToV[xi];
		hwzet=Skew[wzet];
		dum=RPToHomogeneous[hwzet,vzet];
		dum[[4,4]]=0;
		dum,
		Message[gen::wrongD,"TwistToHomogeneous :"]]];
		
		
RigidOrientation[g_]:=  (*Converts from g to R*)
	Block[{r1,r2,r3,r},
		If[Dimensions[g]=={4,4},
	
		flatg=Flatten[g];
		r1=Take[flatg,{1,3}];
		r2=Take[flatg,{5,7}];
		r3=Take[flatg,{9,11}];
		r={r1,r2,r3},
		Message[gen::wrongD,"RigidOrientation  :"]]];

RigidPosition[g_]:=     (*Converts from g to p*)
	Block[{p},
		If[Dimensions[g]=={4,4},
		p={g[[1,4]],g[[2,4]],g[[3,4]]},
		Message[gen::wrongD,"RigidPosition  :"]]];
		

RigidTwist[g_]:= (*Converts from g to zeta 6*)
	Block[{r,p,theta,w,forv,forp},
		If[Dimensions[g]=={4,4},
		r=RigidOrientation[g];
		p=RigidPosition[g];
		w=RotationAxis[r];
		theta=Magnitude[w];
		If[theta==0,
		v=p/Magnitude[p],
		w=w/Magnitude[w];
		forv=((IdentityMatrix[3]-Outer[Times,w,w])*Sin[theta]+Skew[w]*Versine[theta]+(Outer[Times,w,w])theta);
		forp=Inverse[forv];
		v=forp.p];
		xi=Flatten[{v,w}],
		Message[gen::wrongD,"RigidTwist  :"]]];
		


TwistPitch[xi_]:=   (*Gives pitch of screw given xi*)
	
	Block[{w,v,h,dim},
		dim=Dimensions[xi];
		If[dim=={6}||dim=={4,4},
		w=XiToW[xi];
		v=XiToV[xi];
		h=w.v/(Magnitude[w])^2,
		Message[gen::wrongD,"TwistPitch   :"]]];
	
	
TwistAxis[xi_]:=(*Given Xi 6 or Xi 4X4 gives axis of screw*)

	Block[{w,v,lxi,dim},
		dim=Dimensions[xi];
		If[dim=={6} || dim=={4,4},
		w=XiToW[xi];
		v=XiToV[xi];
		If[Magnitude[w]!=0,
			lxi=CrossProduct[w,v]/(Magnitude[w])^2,
			lxi=v],
	Message[gen::wrongD,"TwistAxis   :"]]];
	
	
TwistMagnitude[xi_]:=  (*Given Xi 6 or Xi 4X4 gives magnitude of screw*)

	Block[{w,v,val},
		dim=Dimensions[xi];
		If[dim=={4,4} || dim=={6},
		w=XiToW[xi];
		v=XiToV[xi];
		If[Magnitude[w]!=0,val=Magnitude[w],val=Magnitude[v]],
	Message[gen::wrongD,"TwistMagnitude   :"]]];
	
	
	
RigidAdjoint[g_]:= (* Finds Adjoint Matrix given g*)

	Block[{r1,r2,r3,row1,row2,row3,row4,row5,row6,r,p,admat},
		r=RigidOrientation[g];
		p=RigidPosition[g];
		r3={0,0,0};
		r1=Flatten[r];
		r2=Flatten[Skew[p].r];
		row1=Join[Take[r1,3],Take[r2,3]];
		row2=Join[Take[r1,{4,6}],Take[r2,{4,6}]];
		row3=Join[Take[r1,{7,9}],Take[r2,{7,9}]];
		row4=Join[r3,Take[r1,{1,3}]];
		row5=Join[r3,Take[r1,{4,6}]];
		row6=Join[r3,Take[r1,{7,9}]];
		admat={row1,row2,row3,row4,row5,row6}];
		

WrenchTransformation[p_,R_]:=(*Constructs Wrench Transformation Matrix from p and R*)
	Block[{botleft,W},
		botleft=Skew[p] . R;
		W={
		   Join[R[[1]],{0,0,0}],
		   Join[R[[2]],{0,0,0}],
		   Join[R[[3]],{0,0,0}],
		   Join[botleft[[1]],R[[1]]],
		   Join[botleft[[2]],R[[2]]],
		   Join[botleft[[3]],R[[3]]]}];
		   
WrenchTransformation[g_]:=(*Constructs Wrench Transformation Matrix from g*)
	Block[{p,R},
		p=RigidPosition[g];
		R=RigidOrientation[g];
		WrenchTransformation[p,R]];
		   
		   
ContactBasis[ctype_]:=(*Given Contact Type gives basis*)
	Block[{basis,cctype},
		cctype=ToString[ctype];
		Which[
		cctype=="PCNF",
		  basis={0,0,1,0,0,0},
		cctype=="PCWF",
		  basis={{1,0,0},{0,1,0},{0,0,1},{0,0,0},{0,0,0},{0,0,1}},
		cctype=="SF",
		  basis={{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,0},{0,0,0,0},{0,0,0,1}}]];
		  

	  
		
RobotSpatialJacobian[OutXi__]:= (*Constructs the Jacobian for a robot of any no of links*)
	Block[{g,w,v,n,wv,Ad,Es,Jacob,jj,ForTh,Xi},
		Xi={OutXi};
	   	n=Dimensions[Xi][[1]];
		g=TwistExp[Xi[[1,1]],Xi[[1,2]]];
		Ad=Array[0*#&,{6,6}];
		For[jj=1,jj<=n,jj++,
			Es=Xi[[jj,1]];
			ForTh=Xi[[jj,2]];
			If[jj==1,Jacob=Partition[Es,6]];
			If[jj>1,
				If[jj>2,g=g.TwistExp[Xi[[jj-1,1]],Xi[[jj-1,2]]]];
				Ad=RigidAdjoint[g];
				Es=Ad.Es;
			Jacob=Join[Jacob,Partition[Es,6]]]];
			Jacob=Transpose[Jacob]];
			
			

StackCols[mats__] :=(* Stack matrix columns together *)
  Block[
    {i,j},
    Table[
      Join[ Flatten[Table[{mats}[[j]][[i]], {j,Length[{mats}]}], 1] ],
      {i, Length[ {mats}[[1]] ]}]
  ];


StackRows[mats__] := (* Stack matrix rows together *)
	Join[Flatten[{mats}, 1]];
	

ZeroMatrix[nr_, nc_] := Table[0, {nr}, {nc}];
ZeroMatrix[nr_] := zeroMatrix[nr, nr];(* Matrix of zeros *)


	
		
End[];
	
	
EndPackage[];
