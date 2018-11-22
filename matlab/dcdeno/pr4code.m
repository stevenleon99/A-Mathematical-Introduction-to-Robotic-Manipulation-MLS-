% PR4CODE.M - code to solve HW3's prob 4

p0 = [-.6424; 0.0123; 0.0172];
p1 = [-.5224; 0.0123; 0.0172];
R = ;  % fill in the matrix
step = 0.1;

ang = []; % store joint angles from inverse kinematics at each time point
for t=0:step:1,
	p = p0 + t*(p1 - p0);
	compute g(t)
	do inverse kinematics;
end

plot the angles vs time;
