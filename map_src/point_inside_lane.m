function  inside =point_inside_lane(p,coordinate, Vertex_index)
% return flag -1 for inside; 1 for outside 


%%
% coordinate = offlineData.coordinate;
% Vertex_index = offlineData.Vertex_index;
% -1 inside +1 outside
line_det = zeros(1,4);

for i=1:4                 %计算当前点在离线地图的哪个边? sideIndex
    P0=[p,1];
    A=[coordinate(Vertex_index(i,1),:),1];
    B=[coordinate(Vertex_index(i,2),:),1];
    line_matrix = [P0;A;B];
    line_det(i)=det(line_matrix);
end

if(line_det(1)>0 && line_det(2)>0 && line_det(3)>0 && line_det(4)>0)
    inside = -1; %inside
else
    inside =  1; %outside
end
    