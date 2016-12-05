
function draw_observations_nav_ground (innerLine_coordinate, innerLine_Vertex_index,...
                                middleLine_coordinate, middleLine_Vertex_index,...
                                outerLine_coordinate, outerLine_Vertex_index,...
                                landmarks, vehicle_state, object_list, ...
                                object_of_interest_id, sensor_data_raw, ...
                                patac_navi, configuration)
%-------------------------------------------------------
% University of Zaragoza
% Centro Politecnico Superior
% Robotics and Real Time Group
% Authors:  J. Neira, J. Tardos
% Date   :  7-2004
%-------------------------------------------------------
%-------------------------------------------------------
% global configuration;

if configuration.step_by_step
    figure(configuration.observations); clf; axis equal; hold on;
    
    configuration.ground = configuration.observations;
    draw_map_nav_ground (innerLine_coordinate,innerLine_Vertex_index,...
                      middleLine_coordinate,middleLine_Vertex_index,...
                      outerLine_coordinate,outerLine_Vertex_index,...
                      landmarks, vehicle_state,patac_navi,configuration);

    % vehicle.x = vehicle_state(1:3);
    % vehicle.P = zeros(3, 3);
    
    % %draw vehicle
    % draw_vehicle(vehicle.x, vehicle.P, 'b', configuration);
    % %arrow as vehicle velocity vector
    % initial_point=vehicle_state(1:2);
    % end_point=initial_point+vehicle_state(4:5);
    % generate_arrow(initial_point, end_point, 'c'); 

    % if nargin == 4 % which is used
    %     [ix, iy, ind] = obs_rows(which);
    %     observations.z = observations.z(ind);
    %     observations.R = observations.R(ind,ind);
    %     observations.m = length(which);
    % end
    
    %draw observations
    draw_obs_nav_ground (object_list,sensor_data_raw,object_of_interest_id,configuration);
    
%     for p = 1:observations.m,
%         if configuration.ellipses
%             draw_ellipse (observations.z(2*p-1:2*p), observations.R(2*p-1:2*p, 2*p-1:2*p), 'g');
%         end
% %         if configuration.tags
% %             ht = text(observations.z(2*p-1)-0, observations.z(2*p)+0.05, ['O' num2str(p)]);
% %             set(ht, 'Color', 'g');
% %         end  
%     end

    % %draw local ground map and local landmarks    
    % plot(observations.local_ground.inner(:,1),  observations.local_ground.inner(:,2),'r.');
    % plot(observations.local_ground.middle(:,1), observations.local_ground.middle(:,2),'y.');    
    % plot(observations.local_ground.outer(:,1),  observations.local_ground.outer(:,2),'r.');   
    
    % for i = 1: length (observations.local_landmarks_id)
    %     if(observations.local_landmarks(i,5)<=60)%Type 10,20, ...,60 is manuveur point
    %         %plot(observations.local_landmarks(i,1), observations.local_landmarks(i,2),'rd', 'MarkSize', 10);
    %         rectangle('Position',observations.local_landmarks(i,1:4),'Curvature',0.2,...
    %                    'FaceColor',[0.5 0 0],'EdgeColor','r','LineWidth',5)
    %     else
    %         %plot(observations.local_landmarks(i,1), observations.local_landmarks(i,2),'rh', 'MarkSize', 10);
    %         rectangle('Position',observations.local_landmarks(i,1:4),'Curvature',1,...
    %                    'FaceColor',[0.5 0 0.5],'EdgeColor','b','LineWidth',2)
    %     end
    % end
    % %title(sprintf('OBSERVATIONS at step %d: %d', step, observations.m));
    % pause
end


