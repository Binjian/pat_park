
function draw_observations_nav_ground (innerLine_coordinate, innerLine_Vertex_index,...
                                middleLine_coordinate, middleLine_Vertex_index,...
                                outerLine_coordinate, outerLine_Vertex_index,...
                                landmarks, landmarks_in_proximity_id_in_front,...
                                landmarks_in_proximity_id_in_rear, vehicle_state, object_list, ...
                                 CIPO_id_lcs, CIPO_id_next_lcs, sensor_data_raw, ...
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
    

    draw_map_nav_lnm (innerLine_coordinate, innerLine_Vertex_index,...
                        middleLine_coordinate, middleLine_Vertex_index,...
                        outerLine_coordinate, outerLine_Vertex_index,configuration);

    visible_front = find(landmarks_in_proximity_id_in_front);
    local_landmarks_id = landmarks_in_proximity_id_in_front(visible_front);
    local_landmarks = landmarks(local_landmarks_id,:) ;
    draw_map_nav_ldm (local_landmarks,2,configuration);

    visible_rear = find(landmarks_in_proximity_id_in_rear);
    local_landmarks_id = landmarks_in_proximity_id_in_rear(visible_rear);
    local_landmarks = landmarks(local_landmarks_id,:) ;
    draw_map_nav_ldm (local_landmarks,0.5,configuration);
    

    %draw vehicle
    vehicle.x = vehicle_state(1:3);
    vehicle.P = zeros(3, 3);        
    draw_vehicle(vehicle.x, vehicle.P, 'b', configuration);
    %arrow as vehicle velocity vector
    initial_point = vehicle_state(1:2);
    end_point = initial_point+vehicle_state(4:5);
    generate_arrow(initial_point, end_point, 'r');

    %draw navigation list;
    x = patac_navi(:,1);%observations.z(1:2:end);
    y = patac_navi(:,2);%observations.z(2:2:end);
    plot(x, y, 'go','MarkerSize',10);

    configuration.ground = configuration.observations;
     
    %draw observations
    draw_obs_nav_ground (object_list,sensor_data_raw,CIPO_id_lcs, CIPO_id_next_lcs,configuration);
    
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


