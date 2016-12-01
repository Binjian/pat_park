

function draw_map_nav_ground (innerLine_coordinate, innerLine_Vertex_index,...
                                middleLine_coordinate, middleLine_Vertex_index,...
                                outerLine_coordinate, outerLine_Vertex_index,...
                                landmarks, vehicle_state, patac_navi,configuration)
%-------------------------------------------------------
% University of Zaragoza
% Centro Politecnico Superior
% Robotics and Real Time Group
% Authors:  J. Neira, J. Tardos
% Date   :  7-2004
%-------------------------------------------------------
%-------------------------------------------------------
%global configuration;

figure(configuration.ground); clf; axis equal; hold on;
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

% for i = 1: size (landmarks,1)
%     if(landmarks(i,5)<=80)%Type 10,20, ...,60 is manuveur point
%         plot(landmarks(i,[1 3]), landmarks(i,[2 4]),'-ro',...
%              'LineWidth',2,...
%              'MarkerSize', 2,...
%              'MarkerEdgeColor','c',...
%              'MarkerFaceColor','c');
%     %     rectangle('Position',[landmarks(i,1:4), ,'Curvature',0.2,...
%     %                'FaceColor',[0.5 0 0],'EdgeColor','r','LineWidth',5)
%     else
%         plot(landmarks(i,[1 3]), landmarks(i,[2 4]),'--yo',...
%              'LineWidth',2,...
%              'MarkerSize', 2,...
%              'MarkerEdgeColor','b',...
%              'MarkerFaceColor','y');
%         % rectangle('Position',landmarks(i,1:4),'Curvature',1,...
%         %            'FaceColor',[0.5 0 0.5],'EdgeColor','b','LineWidth',2);
%     end
% end

for i = 1:size (landmarks,1)
    switch(landmarks(i,5))    %地标中颜色符号表示可在下方修�?
    case (120)   
        color = '*b';    %groundarrow = groundarrow + 1;
    case(62)   
        color = 'g';    %leftturn_start = leftturn_start + 1;
    case(61)   
        color = 'm';    %leftturn_stop = leftturn_stop + 1;
    case(90) 
        color = 'm';    %parkslotoffroad = parkslotoffroad + 1;
    case(82) 
        color = 'c';    %parkslotonroad_start = parkslotonroad_start + 1;
    case(81)
        color = 'c';    %parkslotonroad_stop = parkslotonroad_stop + 1;
    case(110) 
        color = '*r';    %speedLimit = speedLimit + 1;
    case(22)
        color = 'y';    %tcrossing_start = tcrossing_start + 1;
    case(21) 
        color = 'y';    %tcrossing_stop = tcrossing_stop + 1;
    case(130)
        color = '*g';    %trafficLight = trafficLight + 1;
    case(30)
        color = 'k';    %zcrossing = zcrossing + 1;
    otherwise
        color = '*y';    %UnClassified (180)
    end
    
    plot([landmarks(i,1),landmarks(i,3)],[landmarks(i,2),landmarks(i,4)],color);
end


vehicle.x = vehicle_state(1:3);
vehicle.P = zeros(3, 3);
    
%draw vehicle
draw_vehicle(vehicle.x, vehicle.P, 'b', configuration);
%arrow as vehicle velocity vector
initial_point = vehicle_state(1:2);
end_point = initial_point+vehicle_state(4:5);
generate_arrow(initial_point, end_point, 'r');


%draw navigation list;
x = patac_navi(:,1);%observations.z(1:2:end);
y = patac_navi(:,2);%observations.z(2:2:end);
plot(x, y, 'go','MarkerSize',20);

% trajectory(1).x = [0 0 0]';
% trajectory(1).P = zeros(3, 3);
% for step = 1 : length(ground.motion),
%     [trajectory(step+1).x, trajectory(step+1).P] = ...
%         ucomp(trajectory(step).x, trajectory(step).P, ...
%         ground.motion(step).x, zeros(3,3));
% end
% draw_trajectory (trajectory, 'r');

%title(sprintf('GROUND TRUTH, features: %d', ground.n));

