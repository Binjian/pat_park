function  [projection_dist, inside] =point_dist2lane(p,coordinate, Vertex_index)
% return flag -1 for inside; 1 for outside 


%%
% coordinate = offlineData.coordinate;
% Vertex_index = offlineData.Vertex_index;
% -1 inside +1 outside
line_det = zeros(1,4);
projection_dist = zeros(1,4);

for i=1:4                 
    P1 = coordinate(Vertex_index(i,1),:);
    P2 = coordinate(Vertex_index(i,2),:);
    Q=[p,1];
    A=[P1,1];
    B=[P2,1];
    line_det(i)=det([Q;A;B]);
    projection_dist(i) = abs(line_det(i))/norm(P1-P2);
end

if(line_det(1)>0 && line_det(2)>0 && line_det(3)>0 && line_det(4)>0)
    inside = -1; %inside
else
    inside =  1; %outside
end


    