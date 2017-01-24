%-------------------------------------------------------
% Patac
% SHD
% Software
% Authors:  Binjian Xin
% Date   :  2017-1
%-------------------------------------------------------
function  inside =point_inside_lane_offset(p,coordinate, Vertex_index,offset)
% return flag -1 for inside; 1 for outside 


%%
% coordinate = offlineData.coordinate;
% Vertex_index = offlineData.Vertex_index;
% -1 inside +1 outside
line_det = zeros(1,4);
projection_dist = zeros(1,4);

for i=1:4                 %计算当前点在离线地图的哪个边�? sideIndex
    P1 = coordinate(Vertex_index(i,1),:);
    P2 = coordinate(Vertex_index(i,2),:);
    A=[P1,1];
    B=[P2,1];
    P0=[p,1];
    line_matrix = [P0;A;B];
    line_det(i)=det(line_matrix);
    projection_dist(i) = abs(line_det(i))/norm(P1-P2);    
end

if(line_det(1)>0 && line_det(2)>0 && line_det(3)>0 && line_det(4)>0 &&...
        projection_dist(1)>offset && projection_dist(2)>offset && projection_dist(3)>offset && projection_dist(4)>offset)
    inside = -1; %inside
else
    inside =  1; %outside
end
    