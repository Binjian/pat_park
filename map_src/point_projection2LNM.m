%-------------------------------------------------------
% Patac
% SHD
% Software
% Authors:  Binjian Xin
% Date   :  2017-1
%-------------------------------------------------------
function  [projected_point_veh, dist2StartCorner] =point_projection2LNM(p, corner_1, corner_2)
% return flag -1 for inside; 1 for outside 
x0 = p(1);
y0 = p(2);
x1 = corner_1(1);
y1 = corner_1(2);
x2 = corner_2(1);
y2 = corner_2(2);


A = [x2-x1, y2-y1;...
     y1-y2, x2-x1];
B = -[-x0*(x2-x1)-y0*(y2-y1);...
      -y1*(x2-x1)+x1*(y2-y1)];
projected_point_veh = linsolve(A,B);
distance_vec = projected_point_veh-corner_1';
dist2StartCorner = norm(distance_vec);
