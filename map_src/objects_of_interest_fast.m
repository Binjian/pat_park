function [obj_closest_in_path_ID, obj_closest_in_next_path_ID, ego_location] = objects_of_interest_fast(object_list,object_num,...
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
    switch(sum(inside))%couterclockwise
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

if( ~(ego_location==2 || ego_location==1) ) 
    obj_closest_in_path_ID = 0;
    obj_closest_in_next_path_ID = 0;
    return;
end
if(ego_location == 1)
    next_lane = 2;
else
    next_lane = 1;
end  


[projection_dist,~]=point_dist2lane(p,outerLine_coordinate,outerLine_Vertex_index);
[~,sideIndex]=min(projection_dist);

%find the projected point of vehicle on the outer lane mark and its distance to the 1st corner.
corner_1=outerLine_coordinate(outerLine_Vertex_index(sideIndex,1),:);
corner_2=outerLine_coordinate(outerLine_Vertex_index(sideIndex,2),:);
[~, dist2StartCorner_veh] = point_projection2LNM(p, corner_1, corner_2);


%find object in ego lane and next lane
object_list_lane_loc = object_list(1:object_num,9) ;

objects_in_ego_lane_id = find(object_list_lane_loc == ego_location);
objects_in_next_lane_id = find( object_list_lane_loc == next_lane);

% objects_in_ego_lane_id(1:size(objects_in_next_lane_id_temp,1))=objects_in_ego_lane_id_temp;
max_len = max(size(objects_in_ego_lane_id,1),size(objects_in_next_lane_id,1));
dist2StartCorner_obj      = zeros(max_len,2);

if(size(objects_in_ego_lane_id,1)~=0)
    objects_in_ego_lane  =  [object_list(objects_in_ego_lane_id,2),object_list(objects_in_ego_lane_id,4)];
    obj_closest_in_path_ID_dist = dist2StartCorner_veh+500;
    obj_closest_in_path_ID = 0;
    %Find the closest in-path object (CIPO)
    for  i = 1:size(objects_in_ego_lane,1)
        p = objects_in_ego_lane(i,:);
        [~,dist2StartCorner_obj(i,1)]=point_projection2LNM(p, corner_1, corner_2); % projection onto the outerlane 

        if (dist2StartCorner_obj(i,1) >= dist2StartCorner_veh && ...
                dist2StartCorner_obj(i,1) <= obj_closest_in_path_ID_dist)
            obj_closest_in_path_ID = objects_in_ego_lane_id(i);
            obj_closest_in_path_ID_dist = dist2StartCorner_obj(i,1);
        end
    end
else
    obj_closest_in_path_ID = 0;
end

% obj_closest_in_next_path_ID =0;
if(size(objects_in_next_lane_id,1)~=0)
    objects_in_next_lane =  [object_list(objects_in_next_lane_id,2),object_list(objects_in_next_lane_id,4)];
    obj_closest_in_path_ID_dist = dist2StartCorner_veh+500;
    obj_closest_in_next_path_ID = 0;

    %Find the closest Next Lane object
    for  i = 1:size(objects_in_next_lane,1)
        p = objects_in_next_lane(i,:);
        [~, dist2StartCorner_obj(i,2)] = point_projection2LNM(p, corner_1, corner_2);
        if (dist2StartCorner_obj(i,2) > dist2StartCorner_veh && ...
                dist2StartCorner_obj(i,2) < obj_closest_in_path_ID_dist)
            obj_closest_in_next_path_ID = objects_in_next_lane_id(i);
            obj_closest_in_path_ID_dist = dist2StartCorner_obj(i,2);
        end
     end    
else
    obj_closest_in_next_path_ID = 0;
end
%Sort objects in ego and next lane in driving direction and in the proximity sequence of ego vehicle.





%flipud/fliplr/flip(a,dim);
%sortrows
% object_of_interest_id = obj_closest_in_path_ID;
