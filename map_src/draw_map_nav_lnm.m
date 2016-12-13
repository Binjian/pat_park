

function draw_map_nav_lnm (innerLine_coordinate, innerLine_Vertex_index,...
                                middleLine_coordinate, middleLine_Vertex_index,...
                                outerLine_coordinate, outerLine_Vertex_index,...
                                configuration)
%-------------------------------------------------------
% University of Zaragoza
% Centro Politecnico Superior
% Robotics and Real Time Group
% Authors:  J. Neira, J. Tardos
% Date   :  7-2004
%-------------------------------------------------------
%-------------------------------------------------------
%global configuration;

% figure(configuration.ground); clf; axis equal; hold on;
plot(innerLine_coordinate(:,1), innerLine_coordinate(:,2), 'r-');
plot(middleLine_coordinate(:,1), middleLine_coordinate(:,2), 'y-');
plot(outerLine_coordinate(:,1), outerLine_coordinate(:,2), 'r-');

for i=1:4
    A=innerLine_coordinate(innerLine_Vertex_index(i,1),:);
    B=innerLine_coordinate(innerLine_Vertex_index(i,2),:);
    plot(A(1),A(2),'r.','MarkerSize',20);
    plot(B(1),B(2),'r.','MarkerSize',20);
    A=middleLine_coordinate(middleLine_Vertex_index(i,1),:);
    B=middleLine_coordinate(middleLine_Vertex_index(i,2),:);
    plot(A(1),A(2),'y.','MarkerSize',20);
    plot(B(1),B(2),'y.','MarkerSize',20);    
    A=outerLine_coordinate(outerLine_Vertex_index(i,1),:);
    B=outerLine_coordinate(outerLine_Vertex_index(i,2),:);
    plot(A(1),A(2),'g.','MarkerSize',20);
    plot(B(1),B(2),'g.','MarkerSize',20);
end

