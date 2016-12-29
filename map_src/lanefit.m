
%right_shift = 2; assuming middle lane mark is used, this parameter gives
%the right shift of line fitting result.
%point_num = 50; the point of number used for line fitting. assuming point
%interval is 1m, then the curve fitting result is 50m
%middleLine is the middle lane mark to be fitted
%flag is for clockwise (1) or couter-clockwise (-1)
%ang is yaw angle in Radian referring to east of GCS
%x0 y0 is the vehicle locaton referring to the GCS

% x0 =  ; y0 =   ;
% flag =   ;    0逆时针, 1顺时针 
% ang = pi;   x和原坐标系横轴逆时针方向上的夹角，x为车辆行驶方向 
function cubic_poly_coef  = lanefit(veh_state, flag, middleLine, outerLine, right_shift, ...
  lane_pos, point_num)

%Find the closest side
p =  veh_state(1:2)';
[projection_dist,~]=point_dist2lane(p,outerLine.coordinate,outerLine.Vertex_index);
[~,sideIndex]=min(projection_dist);
%find the projected point of vehicle on the outer lane mark and its distance to the 1st corner.
corner_1=outerLine.coordinate(outerLine.Vertex_index(sideIndex,1),:);
corner_2=outerLine.coordinate(outerLine.Vertex_index(sideIndex,2),:);
[~, dist2StartCorner_veh] = point_projection2LNM(p, corner_1, corner_2);
start_id = middleLine.Vertex_index(sideIndex,1);
end_id = middleLine.Vertex_index(sideIndex,2);
% n = size(side_lane,1);
side_lane_dist = middleLine.coordinate(start_id:end_id,3);
% cor_index = find(side_lane_dist<dist2StartCorner_veh,n,'last');
lanepoint_behind = find(side_lane_dist<dist2StartCorner_veh);
cor_index = max(lanepoint_behind);
cor_index = cor_index + middleLine.Vertex_index(sideIndex,1)-1;

 %  é?‰??–?–???‘ 
 if flag <0.5
      side_num_after = middleLine.Vertex_index(sideIndex,2) - cor_index;
      side_num_before = cor_index - middleLine.Vertex_index(sideIndex,1);
      if (side_num_after >= point_num && side_num_before>20)
          mid_insight = middleLine.coordinate(cor_index : (cor_index+point_num) , 1 : 2);  
      else
          cubic_poly_coef = zeros(1,4);
          return;
      end
      
 else
     side_num_before = middleLine.Vertex_index(sideIndex,2) - cor_index;
     side_num_after = cor_index - middleLine.Vertex_index(sideIndex,1);
     
     if (side_num_after >= point_num && side_num_before>20)
          mid_insight = middleLine.coordinate((cor_index-point_num) : cor_index , 1 : 2);
     else
          cubic_poly_coef = zeros(1,4);
          return;
     end
 end

 mid_insight(:,1) = mid_insight(:,1) - veh_state(1);
 mid_insight(:,2) = mid_insight(:,2) - veh_state(2);
 Tab = [ 0; 0; -veh_state(3)];
 mid_insight2 = (tpcomp(Tab, mid_insight'))';

 %curve fitting
 cubic_poly_coef = polyfit(mid_insight2(:,1),mid_insight2(:,2),3); 
 %   y(i) = C(1)*x(i)*x(i)*x(i)+ C(2)*x(i)*x(i) + C(3)*x(i) + C(4);
 
 %right shift
 if(lane_pos == 2 && flag <0.5 || lane_pos == 1 && flag >0.5 )
    shift_coef_4 = right_shift*sqrt(cubic_poly_coef(3)*cubic_poly_coef(3)+1);
 elseif(lane_pos == 2 && flag >0.5 || lane_pos == 1 && flag <0.5)
    shift_coef_4 = -right_shift*sqrt(cubic_poly_coef(3)*cubic_poly_coef(3)+1);
 else
    shift_coef_4 = 0;
 end

 cubic_poly_coef(4) =cubic_poly_coef(4) - shift_coef_4;

