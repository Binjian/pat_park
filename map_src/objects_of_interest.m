function [obj_closest_in_path_ID, obj_closest_in_next_path_ID, ego_location] = objects_of_interest(object_list,object_num,...
	x_in_lcs, y_in_lcs, ...
	innerLine_coordinate, innerLine_Vertex_index, ...
	middleLine_coordinate, middleLine_Vertex_index,...
	outerLine_coordinate, outerLine_Vertex_index,...
	 option)

inside 		= zeros(1,3);
p = [x_in_lcs,y_in_lcs];
inside(1) = point_inside_lane(p,innerLine_coordinate, innerLine_Vertex_index);
inside(2) = point_inside_lane(p,middleLine_coordinate, middleLine_Vertex_index);
inside(3) = point_inside_lane(p,outerLine_coordinate, outerLine_Vertex_index);

if(option.clockWise == 0)%counter-clockwise
    switch(sum(inside))
        case  3  %( 1  1  1) outside outer ring what if in case of on the line, the output could be zero?!
        	ego_location = 3; 
        case  1  %( 1  1 -1)
        	ego_location = 2; %outer lane 
        case -1  %( 1 -1 -1)
        	ego_location = 1; %inner lane
        case -3  %(-1 -1 -1)
        	ego_location = 0; %inside inner ring
        otherwise
            ego_location = 5;%warning('Unexpected location!')
    end
else%clockwise
    switch(sum(side_direction_Ego))%couterclockwise
        case -3 %( -1  -1  -1)
        	ego_location = 3; 
        case -1 %( -1  -1   1)
        	ego_location = 2; %outer lane 
        case  1 %( -1   1   1)
         	ego_location = 1; %inner lane
        case  3 %(  1   1   1)
        	ego_location = 0; %inside inner ring
        otherwise
            ego_location = 5;%warning('Unexpected location!')
    end
end


%find object in ego lane and next lane
object_list_lane_loc = object_list(1:object_num,9) ;
objects_in_ego_lane_id = find(object_list_lane_loc == ego_location);
objects_in_next_lane_id = find( object_list_lane_loc ~= ego_location ...
                              &  object_list_lane_loc ~= 3 ...
                              &  object_list_lane_loc ~= 0);

% objects_in_ego_lane_id(1:size(objects_in_next_lane_id_temp,1))=objects_in_ego_lane_id_temp;
max_len = max(size(objects_in_ego_lane_id,1),size(objects_in_next_lane_id,1));
lat_distance_obj        = zeros(max_len,2);
side_direction_obj      = zeros(max_len,2);
nearestIndex_obj        = zeros(max_len,2);

if(size(objects_in_ego_lane_id,1)~=0)
    objects_in_ego_lane  =  [object_list(objects_in_ego_lane_id,2),object_list(objects_in_ego_lane_id,4)];
    %Initialize search ego-lane
    p = objects_in_ego_lane(1,:);%object_list(objects_in_ego_lane_id(1),2:3); %
    [lat_distance_obj(1,1),side_direction_obj(1,1),nearestIndex_obj(1,1)] = calDistance(p,middleLine_coordinate,middleLine_Vertex_index,option);
    obj_closest_in_path_ID_Index = nearestIndex_obj(1,1);
    obj_closest_in_path_ID = objects_in_ego_lane_id(1);

    %Find the closest in-path object (CIPO)
    for  i = 2:size(objects_in_ego_lane,1)
        p = objects_in_ego_lane(i,:);
        [lat_distance_obj(i,1),side_direction_obj(i,1),nearestIndex_obj(i,1)]=calDistance(p,middleLine_coordinate,middleLine_Vertex_index,option); % projection onto the middleLine
        if(option.clockWise == 0) %counter-clockwise
            if (nearestIndex_obj(i,1) > nearestIndex_Ego(2) && ...%middleLine Index-->(2);in front of the ego car;
                    nearestIndex_obj(i,1) < obj_closest_in_path_ID_Index)%nearer than the closest so far
                obj_closest_in_path_ID = objects_in_ego_lane_id(i);
                obj_closest_in_path_ID_Index = nearestIndex_obj(i,1);
            end
        else %clockwise
            if (nearestIndex_obj(i,1) < nearestIndex_Ego(2) && ...%middleLine Index-->(2);in front of the ego car; clockwise is smaller Index --> "<"
                    nearestIndex_obj(i,1) > obj_closest_in_path_ID_Index)%nearer than the closest so far, clockwise is greater Index --> ">"
                obj_closest_in_path_ID = objects_in_ego_lane_id(i);
                obj_closest_in_path_ID_Index = nearestIndex_obj(i,1);
            end
        end
    end
else
    obj_closest_in_path_ID = 0;
end

if(size(objects_in_next_lane_id,1)~=0)
    objects_in_next_lane =  [object_list(objects_in_next_lane_id,2),object_list(objects_in_next_lane_id,4)];
    %Initialize search next-lane
    p = objects_in_next_lane(1,:);
    [lat_distance_obj(1,2),side_direction_obj(1,2),nearestIndex_obj(1,2)] = calDistance(p,middleLine_coordinate,middleLine_Vertex_index,option);
    obj_closest_in_next_path_ID_Index = nearestIndex_obj(1,2);
    obj_closest_in_next_path_ID = objects_in_next_lane_id(1);

    %Find the closest Next Lane object
    for  i = 2:size(objects_in_next_lane,1)
        p = objects_in_next_lane(i,:);
        [lat_distance_obj(i,2),side_direction_obj(i,2),nearestIndex_obj(i,2)]=calDistance(p,middleLine_coordinate,middleLine_Vertex_index,option); % projection onto the middleLine
        if(option.clockWise == 0) %counter-clockwise
            if (nearestIndex_obj(i,2) > nearestIndex_Ego(2) && ...
                    nearestIndex_obj(i,2) < obj_closest_in_next_path_ID_Index)
                obj_closest_in_next_path_ID = objects_in_next_lane_id(i);
                obj_closest_in_next_path_ID_Index = nearestIndex_obj(i,2);
            end
        else %clockwise
            if (nearestIndex_obj(i,2) < nearestIndex_Ego(2) && ...
                    nearestIndex_obj(i,2) > obj_closest_in_next_path_ID_Index)
                obj_closest_in_next_path_ID = objects_in_next_lane_id(i);
                obj_closest_in_next_path_ID_Index = nearestIndex_obj(i,2);
            end
        end
    end    
else
    obj_closest_in_next_path_ID = 0;
end
%Sort objects in ego and next lane in driving direction and in the proximity sequence of ego vehicle.





%flipud/fliplr/flip(a,dim);
%sortrows
% object_of_interest_id = obj_closest_in_path_ID;
