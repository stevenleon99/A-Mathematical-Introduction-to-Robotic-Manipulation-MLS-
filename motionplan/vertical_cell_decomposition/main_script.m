clear
close all

CB={[0 50 50 0; 25 25 50 50],[80 80 70 70; 50 100 100 50]};
qI=[0.5; 0.5];
qG=[95; 95];

V = potential_field(qI,qG,CB);

plot(qI(1, 1),qI(2, 1),'d','MarkerFaceColor','green')
axis equal
hold on
plot(qG(1, 1),qG(2, 1),'s','MarkerFaceColor','red')
for i = 1 : size(CB,2)
    Cpatch = CB{1, i};
    patch(Cpatch(1, :), Cpatch(2, :),'yellow')
end

for i = 1 : size(V, 2)-1   
    plot([V(1, i) V(1, i + 1)], [V(2, i) V(2, i + 1)],'r-','LineWidth',1)   
    hold on
end