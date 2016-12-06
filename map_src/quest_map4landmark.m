function [landmarks_in_proximity_id_in_front, landmarks_in_proximity_id_in_rear] = quest_map4landmark (x_in_lcs, y_in_lcs, landmarks, middleLine_coordinate, middleLine_Vertex_index,...
	configuration,option)
%-------------------------------------------------------
% University of Zaragoza
% Centro Politecnico Superior
% Robotics and Real Time Group
% Authors:  J. Neira, J. Tardos
% Date   :  7-2004
% ground and veh_pose for setting observation.ground
% landmarks and landmarks_in_proximity in column vector.
%-------------------------------------------------------
%-------------------------------------------------------


% landmarks_in_proximity = zeros(size(landmarks));
%Landmarks
x1 = (double(landmarks(:,1))+double(landmarks(:,3)))/2 - ones(size(landmarks,1),1)*x_in_lcs;%landmarks column vector
y1 = (double(landmarks(:,2))+double(landmarks(:,4)))/2 - ones(size(landmarks,1),1)*y_in_lcs;
%visible = find( (abs(x1) <= roi.x) & (abs(y1) <= roi.y) );
distance2ego = (x1.^2 + y1.^2);
[min_dist, ind] = min(distance2ego);
visible = find( (x1.^2 + y1.^2) <= configuration.proximity*configuration.proximity);



%%%%%%%%%%%%%%%%%%%%
p = [x_in_lcs,y_in_lcs];
[lat_distance_Ego,side_direction_Ego,nearestIndex_Ego] = calDistance(p,middleLine_coordinate,middleLine_Vertex_index,option);

if(size(visible,1)~=0)
	landmarks_in_proximity = landmarks(visible,:);
	lat_distance_ldm        = zeros(size(visible,1),1);
	side_direction_ldm      = zeros(size(visible,1),1);
	nearestIndex_ldm        = zeros(size(visible,1),1);

    %Initialize search
%     p = (landmarks_in_proximity(1,1:2)+ landmarks_in_proximity(1,3:4))/2; %
%     [lat_distance_ldm(1),side_direction_ldm(1),nearestIndex_ldm(1)] = calDistance(p,middleLine_coordinate,middleLine_Vertex_index,option);
%     ldm_closest_in_path_ID_Index = nearestIndex_ldm(1);
    ldm_closest_in_path_ID = -1;
    ldm_closest_in_path_ID_Index = nearestIndex_Ego;

    %Find the closest landmark in front
    for  i = 1:size(visible,1)
        p = (landmarks_in_proximity(i,1:2)+ landmarks_in_proximity(i,3:4))/2; %
        [lat_distance_ldm(i),side_direction_ldm(i),nearestIndex_ldm(i)]=calDistance(p,middleLine_coordinate,middleLine_Vertex_index,option); % projection onto the middleLine
        if(option.clockWise == 0) %counter-clockwise
            if (nearestIndex_ldm(i) > nearestIndex_Ego && ...%middleLine Index-->(2);in front of the ego car;
                    (nearestIndex_ldm(i) < ldm_closest_in_path_ID_Index...
                     || ldm_closest_in_path_ID <0 )...
                )%nearer than the closest so far
                ldm_closest_in_path_ID = i;
                ldm_closest_in_path_ID_Index = nearestIndex_ldm(i);
            end
        else %clockwise
            if (nearestIndex_ldm(i) < nearestIndex_Ego && ...%middleLine Index-->(2);in front of the ego car; clockwise is smaller Index --> "<"
                    (nearestIndex_ldm(i) > ldm_closest_in_path_ID_Index...
                     || ldm_closest_in_path_ID <0)...
                )%nearer than the closest so far, clockwise is greater Index --> ">"
                ldm_closest_in_path_ID = i;
                ldm_closest_in_path_ID_Index = nearestIndex_ldm(i);
            end
        end
    end

    landmarks_in_proximity_id_in_front = zeros(size(landmarks,1),1);
    landmarks_in_proximity_id_in_rear  = zeros(size(landmarks,1),1); 
	if(ldm_closest_in_path_ID<0)%no front landmarks all are behind the ego vehicle.
		if(option.clockWise == 0) %counter-clockwise
            landmarks_in_proximity_id_in_rear(1:size(visible,1)) = visible;
        else%clockwise
            landmarks_in_proximity_id_in_rear(1:size(visible,1)) = flip(visible);
        end 
    elseif(abs(ldm_closest_in_path_ID-1)<1e-3)
		landmarks_in_proximity_id_in_front(1:size(visible,1)) = visible;
	else% (ldm_closest_in_path_ID>1)
		if(option.clockWise == 0) %counter-clockwise
			front_size = size(visible,1) - ldm_closest_in_path_ID+1;
			rear_size = ldm_closest_in_path_ID-1;
			landmarks_in_proximity_id_in_front(1:front_size) = visible(ldm_closest_in_path_ID:end);
			landmarks_in_proximity_id_in_rear(1:rear_size)= visible(1:ldm_closest_in_path_ID-1);
		else%clockwise
			front_size = ldm_closest_in_path_ID;
			rear_size = size(visible,1) - ldm_closest_in_path_ID;
			landmarks_in_proximity_id_in_front(1:front_size) = flip(visible(1:ldm_closest_in_path_ID));
			landmarks_in_proximity_id_in_rear(1:rear_size)= flip(visible(ldm_closest_in_path_ID+1:end));
		end
	end
else
    landmarks_in_proximity_id_in_front = zeros(size(landmarks,1),1);
    landmarks_in_proximity_id_in_rear  = zeros(size(landmarks,1),1); 
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




