% Created by Jiaming Zhang on March 10, 2023
% @ Johns Hopkins University

% This script aims to test the Minkowski Sum Algorithm

% Test Case
% This script is used for checking whether your implementation is correct.
% You can modify any of the components if necessary.
clf
A = [1 0 -1; -1 1 -1];

B1 = [1 2 2 1; 1 1 2 2];
B2 = [0 2 1; -6 -6 -4];
B3 = [-1 0 1; 1 -1 1]+6;
B4 = [-1 1 1 -1; -1 -1 1 1] + [7; 0];
B5 = [2 5 5 2; 1 1 3 3] - [8; 0];
B = {B1, B2, B3, B4, B5};
q = [0; 0; 0*pi/180];
% q = [0; 0; 30*pi/180];
% q = [0; 0; 180*pi/180];
% q = [0; 0; 330*pi/180];
transA = -1 * Transform(A,q);
for  i = 1:length(B)
    CB{i} = fn_c_obstacles(B{i}, A, q);
end
figure(2)
hold on 
patch(CB{1}(1,:),CB{1}(2,:),'g-');
patch(CB{2}(1,:),CB{2}(2,:),'g-');
patch(CB{3}(1,:),CB{3}(2,:),'g-');
patch(CB{4}(1,:),CB{4}(2,:),'g-');
patch(CB{5}(1,:),CB{5}(2,:),'g-');
axis equal