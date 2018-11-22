trigexpand = { Sin[x_]^2 -> (1 - Cos[x]^2) };
trigreduce = { (1 - Cos[x_]^2) -> Sin[x]^2 };

trigSimplify[expr_] := Simplify[expr /. trigexpand, Trig->False] /. trigreduce;
