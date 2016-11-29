%-------------------------------------------------------
% PATAC
% ADU
% SHD
% 
% Authors:  Binjian Xin
% Date   :  11-2016
%-------------------------------------------------------
% slam, explore data association algorithms
% hd_map: offline HD map orginal ground
% odo_motion: vehicle odometer data: odo_motion.x; odo_motion.y; odo_motion.yaw;
% sensor_data_raw: raw sensor data 40x4 [ID, x, y, MoveInfo]
% sensor_data_raw.srr[4] srr(1)-srr(4) are four SRR Radar raw data.
% srr(i).Measure --> [rho tita;...]
% srr(i).Rn --> [rho tita;...]c
% sensor_data_fusion: sensor fuison data; sensor_data_fusion
% 127x26
% [ID,RangeX,VelX,RangeY,VelY,MoveInfo,Width,Type,Fus_State,Confidence,
% P11,...P44]; FuseState Byte 9 for Lane Position Judge
% hd_map offline hd map, supposed to be converted to RTM 51r : wgs2utm(Lat,Lon,51,'N');
% hd_map.lanemarking1/lanemarking2/lanemarking3: three lanes, points in counterclockwise order; 1
% for left, 2 for middle, 3 for right;
% hd_map.landmark(:,5) (x,y,length, width, LandMarkType)
% rtk_gps: rtk_gps.[lat, lon, alt, yaw, pitch, roll, ts]
% 
%   enumeration
%   XCrossing_Stop (11) % 十字路口停止线
%   XCrossing_Start (12) % 十字路口目标车道进入线
%   TCrossing_Stop (21)%T字路口停止线
%   TCrossing_Start (22)%T字路口目标车道进入线
%   ZCrossing (30)%人行横道线
%   Stop_1 (51)%边线1（沿车道横向边线），摆渡车停车点。停车点可能没有实际的标记，请按估计的停车点选取位置，比如添加2号楼门口的停车点
%   Stop_2 (52)%边线2（沿车道横向边线）
%   LeftTurn_Stop (61)% 左转L形路口停止线
%   LeftTurn_Start (62)% 左转L形路口进入线
%   RightTurn_Stop (71)% 右转L形路口停止线
%   RightTurn_Start (72)% 右转L形路口进入线
%   ParkSlotOnRoad_Stop (81)% 路边停车位边线1（沿车道横向边线）
%   ParkSlotOnRoad_Start (82)% 路边停车位边线2（沿车道横向边线）
%	ParkSlotOffRoad (90)%  路外停车位，只需测量沿道路行驶方向的开始点和停止点
%   RoadChange (100)% 车道宽度变化点，如两点间距离暗示车道宽度，则此时两点选取应保证，两点连线同测量处的车道垂直，或者增加车道宽度的域。
%   SpeedLimit (110)%限速标志，点类型
%   GroundArrow (120)%地上箭头（参考EPM3的箭头位置定义），点类型
%   TrafficLight (130)%交通灯位置，点类型
%   UnClassified (180)%其他
%   end
%-------------------------------------------------------
function [global_landmarks, local_landmarks_id,...
            vehicle_state, object_list_update, object_of_interest_id, ...
            maneuvers, ...
            odo_state_v, ...
            step_out] ...
    = slam_ekf_patac_map_pose_output...
        (chi2_dat, ...
        innerLine, middleLine, outerLine, landmarks,...
        odo_motion_v,...
        rtk_gps_lat, rtk_gps_lon, rtk_gps_yaw, rtk_gps_ts, ...
        sensor_data_raw, object_list, patac_navi)
global chi2;
persistent step veh_origin_pose;
persistent x_in_lcs_last y_in_lcs_last ts_last yaw_last veh_vel_last;
% local_landmarks_ids = zeros(200,1);
if isempty(step)
    step = 1;
    chi2 = chi2_dat;
    veh_origin_pose = struct('x',0,'y',0,'yaw',0,'ts',0);
    proximity = 50;% in meter

    [x1,y1,utmzone,utmhemi] = wgs2utm(rtk_gps_lat,rtk_gps_lon,51,'N');
    
    veh_origin_pose.x = x1;% in Mercator
    veh_origin_pose.y = y1;% in Mercator
    veh_origin_pose.yaw = rtk_gps_yaw;
    veh_origin_pose.ts = rtk_gps_ts;

    %relocate lane markings in LCS, origin is the starting point.
    innerLine.coordinate = innerLine.coordinate - ...
     [ones(size(innerLine.coordinate,1),1).*veh_origin_pose.x,...
      ones(size(innerLine.coordinate,1),1).*veh_origin_pose.y];
    middleLine.coordinate = middleLine.coordinate - ...
     [ones(size(middleLine.coordinate,1),1).*veh_origin_pose.x,...
      ones(size(middleLine.coordinate,1),1).*veh_origin_pose.y];

    outerLine.coordinate = outerLine.coordinate - ...
     [ones(size(outerLine.coordinate,1),1).*veh_origin_pose.x,...
      ones(size(outerLine.coordinate,1),1).*veh_origin_pose.y];
  
    %relocate landmarks in LCS, origin is the starting point.
    landmarks(:,1) = landmarks(:,1) - veh_origin_pose.x;%landmarks in Mercator system!
    landmarks(:,2) = landmarks(:,2) - veh_origin_pose.y;%landmarks in Mercator system!
    landmarks(:,3) = landmarks(:,3) - veh_origin_pose.x;%landmarks in Mercator system!
    landmarks(:,4) = landmarks(:,2) - veh_origin_pose.y;%landmarks in Mercator system!

    x_in_lcs = x1-veh_origin_pose.x;
    y_in_lcs = y1-veh_origin_pose.y;
    
    x_in_lcs_last = x_in_lcs;
    y_in_lcs_last = y_in_lcs;
    ts_last = rtk_gps_ts;
    yaw_last = rtk_gps_yaw;

    %Relocate object list in LCS
    x0_offset = -5.24/2.0;%vehicle length 5.24m
    y0_offset = 0;%vehicle width 1.8m
    %Square One
      veh_pose = [0 0 0]';% x, y, yaw in LCS
    %standing still
      veh_vel = [0 0]';% V_x, V_y in LCS
      
      veh_vel_last = veh_vel;
    vehicle_state = [veh_pose; veh_vel];
    
    object_list(:,2)=object_list(:,2)+ x0_offset + x1 - veh_origin_pose.x;%RangeX in LCS, displacement of VCS to SCS by 2.6m, half of vehicle length
    object_list(:,3)=object_list(:,3)+ veh_vel(1);%VelX
    object_list(:,4)=object_list(:,4)+ y0_offset + y1 - veh_origin_pose.y;%RangeY in LCS
    object_list(:,5)=object_list(:,5)+ veh_vel(2);%VelY    
    
    %find local landmarks

    local_landmarks_id = quest_map4landmark (x_in_lcs, y_in_lcs, landmarks,proximity);

%     sensor_data_raw(:,2)=sensor_data_raw(:,2)+ x0_offset + veh_origin_pose.x;%RangeX, displacement of VCS to SCS by 2.6m, half of vehicle length
%     sensor_data_raw(:,3)=sensor_data_raw(:,3)+ y0_offset + veh_origin_pose.y;%RangeY
    
%    [veh_pose, veh_vel] = slam_ekf_patac (inner, middle, outer, landmarks, ...
%         odo_motion_x, odo_motion_y, odo_motion_yaw, ...
%         rtk_gps_lat, rtk_gps_lon, rtk_gps_yaw, rtk_gps_ts, ...
%         sensor_data_raw,...
%         proximity);
   
  %locate objects in lanes    
  %decide object list location in lanes
    option.firstTime=1;
    option.clockWise=0;  %0 for counter-clockwise，1 for clockwise        
    object_list_update  = object_localization(object_list, innerLine.coordinate,innerLine.Vertex_index,...
                                                          middleLine.coordinate,middleLine.Vertex_index,...
                                                          outerLine.coordinate,outerLine.Vertex_index,...
                                                          option);
    %find the closest object in ego lane (probably CIPV)

    object_of_interest_id = objects_of_interest(object_list,...
        x_in_lcs, y_in_lcs, innerLine.coordinate, innerLine.Vertex_index,...
        middleLine.coordinate, middleLine.Vertex_index,...
        outerLine.coordinate, outerLine.Vertex_index, option);
else
    step = step +1;
    chi2 = chi2_dat;
    proximity = 50;% in meter
    
    [x1,y1,utmzone,utmhemi] = wgs2utm(rtk_gps_lat,rtk_gps_lon,51,'N');
    

    %relocate lane markings in LCS, origin is the starting point.
    innerLine.coordinate = innerLine.coordinate - ...
     [ones(size(innerLine.coordinate,1),1).*veh_origin_pose.x,...
      ones(size(innerLine.coordinate,1),1).*veh_origin_pose.y];
    middleLine.coordinate = middleLine.coordinate - ...
     [ones(size(middleLine.coordinate,1),1).*veh_origin_pose.x,...
      ones(size(middleLine.coordinate,1),1).*veh_origin_pose.y];

    outerLine.coordinate = outerLine.coordinate - ...
     [ones(size(outerLine.coordinate,1),1).*veh_origin_pose.x,...
      ones(size(outerLine.coordinate,1),1).*veh_origin_pose.y];
  
    %relocate landmarks in LCS, origin is the starting point.
    landmarks(:,1) = landmarks(:,1) - veh_origin_pose.x;%landmarks in Mercator system!
    landmarks(:,2) = landmarks(:,2) - veh_origin_pose.y;%landmarks in Mercator system!
    landmarks(:,3) = landmarks(:,3) - veh_origin_pose.x;%landmarks in Mercator system!
    landmarks(:,4) = landmarks(:,2) - veh_origin_pose.y;%landmarks in Mercator system!

    x_in_lcs = x1-veh_origin_pose.x;
    y_in_lcs = y1-veh_origin_pose.y;    
    
    %Relocate object list in LCS
    x0_offset = -5.24/2.0;%vehicle length 5.24m
    y0_offset = 0;%vehicle width 1.8m
    if(rtk_gps_ts - ts_last>1e-3)
        veh_vel = [x_in_lcs-x_in_lcs_last; y_in_lcs-y_in_lcs_last]/(rtk_gps_ts - ts_last);%/1000;%ms-->s
        veh_vel_last = veh_vel;
    else
        veh_vel = veh_vel_last;
    end
    veh_pose = [x_in_lcs y_in_lcs veh_origin_pose.yaw]';% x, y, yaw in LCS
    vehicle_state = [veh_pose; veh_vel];

    
    object_list(:,2)=object_list(:,2)+ x0_offset + x1 - veh_origin_pose.x;%RangeX in LCS, displacement of VCS to SCS by 2.6m, half of vehicle length
    object_list(:,3)=object_list(:,3)+ veh_vel(1);%VelX
    object_list(:,4)=object_list(:,4)+ y0_offset + y1 - veh_origin_pose.y;%RangeY in LCS
    object_list(:,5)=object_list(:,5)+ veh_vel(2);%VelY    
    
    %local landmarks
    local_landmarks_id = quest_map4landmark (x_in_lcs, y_in_lcs, landmarks, proximity);    
    
%     sensor_data_raw(:,2)=sensor_data_raw(:,2)+ x0_offset + veh_origin_pose.x;%RangeX, displacement of VCS to SCS by 2.6m, half of vehicle length
%     sensor_data_raw(:,3)=sensor_data_raw(:,3)+ y0_offset + veh_origin_pose.y;%RangeY
    
%     [veh_pose, veh_vel] = slam_ekf_patac (inner, middle, outer, landmarks, ...
%         odo_motion_x, odo_motion_y, odo_motion_yaw, ...
%         rtk_gps_lat, rtk_gps_lon, rtk_gps_yaw, rtk_gps_ts, ...
%         sensor_data_raw,...
%         proximity);


    %locate objects in lanes    
    %decide object list location in lanes
    option.firstTime=1;
    option.clockWise=0;  % for counter-clockwise，1 for clockwise          
    object_list_update = object_localization(object_list, innerLine.coordinate,innerLine.Vertex_index,...
                                                          middleLine.coordinate,middleLine.Vertex_index,...
                                                          outerLine.coordinate,outerLine.Vertex_index,...
                                                          option);
    x_in_lcs_last = x_in_lcs;
    y_in_lcs_last = y_in_lcs;
    yaw_last = rtk_gps_yaw;
    ts_last = rtk_gps_ts;
    
    object_of_interest_id = objects_of_interest(object_list,...
        x_in_lcs, y_in_lcs, innerLine.coordinate, innerLine.Vertex_index, ...
        middleLine.coordinate, middleLine.Vertex_index,...
        outerLine.coordinate, outerLine.Vertex_index, option);
end

maneuvers = patac_navi;
global_landmarks = landmarks;
odo_state_v = odo_motion_v;
step_out = step;
