

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
draw_map_nav_lnm (innerLine_coordinate, innerLine_Vertex_index,...
                    middleLine_coordinate, middleLine_Vertex_index,...
                    outerLine_coordinate, outerLine_Vertex_index,configuration);

draw_map_nav_ldm (landmarks,0.5,configuration);


vehicle.x = vehicle_state(1:3);
vehicle.P = zeros(3, 3);
    
% %draw vehicle
% draw_vehicle(vehicle.x, vehicle.P, 'b', configuration);
% %arrow as vehicle velocity vector
% initial_point = vehicle_state(1:2);
% end_point = initial_point+vehicle_state(4:5);
% generate_arrow(initial_point, end_point, 'r');


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

