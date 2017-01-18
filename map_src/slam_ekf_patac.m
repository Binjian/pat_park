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
%   ParkSlotOffRoad (90)%  路外停车位，只需测量沿道路行驶方向的开始点和停止点
%   ParkSlotOffRoad_Start (91)%  路外停车位，只需测量沿道路行驶方向的开始点
%   ParkSlotOffRoad_Stop  (92)%  路外停车位，只需测量沿道路行驶方向的停止点
%   RoadChange (100)% 车道宽度变化点，如两点间距离暗示车道宽度，则此时两点选取应保证，两点连线同测量处的车道垂直，或者增加车道宽度的域。
%   SpeedLimit (110)%限速标志，点类型
%   GroundArrow (120)%地上箭头（参考EPM3的箭头位置定义），点类型
%   TrafficLight (130)%交通灯位置，点类型
%   UnClassified (180)%其他
%   end
%-------------------------------------------------------
%             t_in_utm,...
%             x_in_utm,...
%             y_in_utm,...
%             a_in_utm,...
%             delta_t,...
%             x_last,...
%             y_last,...
%             v_last,...
%             veh_ori_pos,...

function [global_landmarks,...
            vehicle_state, ...
            object_list_update, ...
            CIPO_id,...
            CIPO_next_id,...
            landmarks_in_proximity_id_in_front, ...
            landmarks_in_proximity_id_in_rear,...
            maneuvers, ...
            innerLaneMark,...
            middleLaneMark,...
            outerLaneMark,...
            cubic_poly_coef,...
            ego_location, ...
            odo_state_v, ...
            t_in_utm,...
            x_in_utm,...
            y_in_utm,...
            a_in_utm,...
            delta_t,...
            x_last,...
            y_last,...
            v_last,...
            state_flag,...
            veh_ori_pos,...
            step_out] ...
    = slam_ekf_patac_map_pose_output...
        (chi2_dat, ...
        innerLine, ...
        middleLine, ...
        outerLine, ...
        lane_offset,...
        landmarks,...
        odo_motion_v,...
        rtk_gps_lat, ...
        rtk_gps_lon, ...
        rtk_gps_yaw, ...
        rtk_gps_ts, ...
        sensor_data_raw, ...
        object_list, ...
        object_num, ...
        patac_navi)
global chi2;
persistent step veh_origin_pose configuration;
persistent x_in_utm_last y_in_utm_last a_in_utm_last t_in_utm_last a_rtk_offset;
persistent x_in_utm_last_meas y_in_utm_last_meas veh_vel_last wgs_ts_last;
% local_landmarks_ids = zeros(200,1);
persistent innerLine_lcs middleLine_lcs outerLine_lcs landmarks_lcs patac_navi_lcs object_list_lcs object_num_lcs object_list_update_lcs;
persistent landmarks_in_proximity_id_in_front_lcs landmarks_in_proximity_id_in_rear_lcs sensor_data_raw_lcs;
persistent CIPO_id_lcs CIPO_id_next_lcs ego_location_lcs cubic_poly_coef_lcs;
%Convert rtk raw data to longitude, latitude, yaw in radian and ts in
%second
[wgs_lat,wgs_lon,wgs_yaw,wgs_ts_in_sec] = rtkraw2wgs (rtk_gps_lat, rtk_gps_lon, rtk_gps_yaw,rtk_gps_ts);
[x_in_utm,y_in_utm,~,~] = wgs2utm(wgs_lat,wgs_lon,51,'N');
wgs_yaw = 2*pi - wgs_yaw;


%rtk system --> VCS offset
x_rtk_offset = 0;

y_rtk_offset = 0;
a_rtk_offset = 90*pi/180;%rtk calibration info input!
x_in_utm = x_in_utm+x_rtk_offset;
y_in_utm = y_in_utm+y_rtk_offset;
a_in_utm = wgs_yaw + a_rtk_offset;
t_in_utm = wgs_ts_in_sec;

if a_in_utm>2*pi
    a_in_utm = a_in_utm-2*pi;
end

chi2 = chi2_dat;
%{
persistent cnt;
if(isempty(cnt))
    cnt =0;
else
    cnt = cnt+1;
end

if (isempty(step) || cnt<100 )% first cycle
    step = 0;
    state_flag = 0;

    configuration = struct('ellipses',true,'tags',false,'odometry',true, ...
                            'noise',true,'alpha',0.99,'step_by_step',false,...
                            'people',false,'proximity',50, 'ground',1,'map',2,'observations',3,...
                            'compatibility',4,'ground_hypothesis',5,'hypothesis',6,...
                            'tables',7);
    configuration.step_by_step = true;   
    veh_origin_pose = struct('x_in_utm',0,'y_in_utm',0,'a_in_utm',0,'t_in_utm',0);   

    veh_origin_pose.x_in_utm = x_in_utm;% in Mercator
    veh_origin_pose.y_in_utm = y_in_utm;% in Mercator
    veh_origin_pose.a_in_utm = a_in_utm;
    veh_origin_pose.t_in_utm = t_in_utm;
    
    %Square One
    x_in_lcs = 0;
    y_in_lcs = 0;
    a_in_lcs = 0;
    t_in_lcs = 0;
    veh_pose = [0 0 0]';% x, y, yaw in LCS
    %standing still
    veh_vel = [0 0]';% V_x, V_y in LCS
    vehicle_state = [veh_pose; veh_vel];

    x_in_utm_last = x_in_utm;
    y_in_utm_last = y_in_utm;
    a_in_utm_last = a_in_utm;
    t_in_utm_last = t_in_utm;
    wgs_ts_last = wgs_ts_in_sec;
    x_in_utm_last_meas = x_in_utm;
    y_in_utm_last_meas = y_in_utm;
%     a_last = a_in_utm;
    veh_vel_last = [0 0]';

    innerLine_lcs = innerLine;
    middleLine_lcs = middleLine;
    outerLine_lcs = outerLine;
    landmarks_lcs = landmarks;
    patac_navi_lcs = patac_navi;
    object_list_lcs = object_list;
    object_num_lcs = object_num;
    object_list_update_lcs = object_list;
    landmarks_in_proximity_id_in_front_lcs = zeros(size(landmarks,1),1);
    landmarks_in_proximity_id_in_rear_lcs = zeros(size(landmarks,1),1); 
    CIPO_id_lcs = zeros(10,1);
    CIPO_id_next_lcs = zeros(10,1);
    sensor_data_raw_lcs = sensor_data_raw;
    ego_location_lcs = -1;
    
    landmarks_in_proximity_id_in_front = zeros(size(landmarks,1),1);
    landmarks_in_proximity_id_in_rear  = zeros(size(landmarks,1),1); 
    maneuvers = patac_navi;
    global_landmarks = landmarks;
    odo_state_v = odo_motion_v;
    step_out = 0;
    object_list_update = object_list;
    object_of_interest_id = 0;
    ego_location = -1; % due to invalid data from rtk ego locatoin cannot be determined.
    delta_t = 0;
    
    x_last = 0;
    y_last = 0;
    v_last = [0;0];
    veh_ori_pos = [veh_origin_pose.x_in_utm;veh_origin_pose.y_in_utm;veh_origin_pose.a_in_utm;veh_origin_pose.t_in_utm];
    CIPO_id = zeros(10,1);
    CIPO_next_id = zeros(10,1);
    innerLaneMark = innerLine_lcs.coordinate;
    middleLaneMark = middleLine_lcs.coordinate(:,1:2);
    outerLaneMark = outerLine_lcs.coordinate;    
    return;

end   
step = step +1;
if(step==1)
    innerLine_lcs = innerLine;
    middleLine_lcs = middleLine;
    outerLine_lcs = outerLine;
    innerLine_lcs.coordinate = innerLine.coordinate - ...
     [ones(size(innerLine.coordinate,1),1).*veh_origin_pose.x_in_utm,...
      ones(size(innerLine.coordinate,1),1).*veh_origin_pose.y_in_utm];
    middleLine_lcs.coordinate(:,1:2) = middleLine.coordinate(:,1:2) - ...
     [ones(size(middleLine.coordinate,1),1).*veh_origin_pose.x_in_utm,...
      ones(size(middleLine.coordinate,1),1).*veh_origin_pose.y_in_utm];
    outerLine_lcs.coordinate = outerLine.coordinate - ...
     [ones(size(outerLine.coordinate,1),1).*veh_origin_pose.x_in_utm,...
      ones(size(outerLine.coordinate,1),1).*veh_origin_pose.y_in_utm];

    %relocate landmarks in LCS, origin is the starting point.
    landmarks_lcs = landmarks;
    landmarks_lcs(:,1) = landmarks(:,1) - veh_origin_pose.x_in_utm;% landmarks in LCS!
    landmarks_lcs(:,2) = landmarks(:,2) - veh_origin_pose.y_in_utm;% landmarks in LCS!
    landmarks_lcs(:,3) = landmarks(:,3) - veh_origin_pose.x_in_utm;% landmarks in LCS!
    landmarks_lcs(:,4) = landmarks(:,4) - veh_origin_pose.y_in_utm;% landmarks in LCS!

    %relocate navigation maneuver list in LCS, origin is the starting point.
    patac_navi_lcs = patac_navi;
    patac_navi_lcs(:,1) = patac_navi(:,1) - veh_origin_pose.x_in_utm;%navigation maneuver in LCS!
    patac_navi_lcs(:,2) = patac_navi(:,2) - veh_origin_pose.y_in_utm;%navigation maneuver in LCS!

    veh_pose = [0 0 0]';% x, y, yaw in LCS
    %standing still
    veh_vel = [0 0]';% V_x, V_y in LCS
    vehicle_state = [veh_pose; veh_vel];    
    draw_map_nav_ground(innerLine_lcs.coordinate,innerLine_lcs.Vertex_index,...
                      middleLine_lcs.coordinate,middleLine_lcs.Vertex_index,...
                      outerLine_lcs.coordinate,outerLine_lcs.Vertex_index,...
                      landmarks_lcs, vehicle_state, patac_navi_lcs, configuration);

end

delta_t = (t_in_utm - t_in_utm_last);
if(abs(rtk_gps_lat)<1e-6 ...
   || abs(rtk_gps_lon)<1e-6 ...
   || abs(rtk_gps_ts)<1e-6...
   )%if invalid rtk data after initialization halfways due to rtk signal loss
    state_flag = 1; 
    t_in_lcs = (t_in_utm_last +0.01)-veh_origin_pose.t_in_utm;
    x_in_lcs = (x_in_utm_last+veh_vel_last(1)*0.01)-veh_origin_pose.x_in_utm;
    y_in_lcs = (y_in_utm_last+veh_vel_last(2)*0.01)-veh_origin_pose.y_in_utm;
    a_in_lcs = a_in_utm_last;
    veh_vel = veh_vel_last; 

    x_in_utm_last = x_in_utm_last + veh_vel_last(1)*0.01;
    y_in_utm_last = y_in_utm_last + veh_vel_last(2)*0.01;
    a_in_utm_last = a_in_utm;
    t_in_utm_last = t_in_utm_last+0.01;
%         veh_vel_last = veh_vel;
    wgs_ts_last = wgs_ts_in_sec;
%         x_in_utm_last_meas = x_in_utm;
%         y_in_utm_last_meas = y_in_utm;
else
    if(abs(wgs_ts_in_sec - wgs_ts_last)<1e-6)
        state_flag = 2;
        t_in_lcs = (t_in_utm_last +0.01)-veh_origin_pose.t_in_utm;
        x_in_lcs = (x_in_utm_last+veh_vel_last(1)*0.01)-veh_origin_pose.x_in_utm;
        y_in_lcs = (y_in_utm_last+veh_vel_last(2)*0.01)-veh_origin_pose.y_in_utm;
        a_in_lcs = a_in_utm_last;
        veh_vel = veh_vel_last; 

        x_in_utm_last = x_in_utm_last + veh_vel_last(1)*0.01;
        y_in_utm_last = y_in_utm_last + veh_vel_last(2)*0.01;
        a_in_utm_last = a_in_utm;
        t_in_utm_last = t_in_utm_last+0.01;
%             veh_vel_last = veh_vel;
        %wgs_ts_last = wgs_ts_in_sec;
    else
        state_flag = 3;
        t_in_lcs = t_in_utm - veh_origin_pose.t_in_utm;
        x_in_lcs = x_in_utm - veh_origin_pose.x_in_utm;
        y_in_lcs = y_in_utm - veh_origin_pose.y_in_utm;
        a_in_lcs = a_in_utm; %a_in_utm - veh_origin_pose.a_in_utm;
        %veh_vel = [(x_in_lcs-x_in_lcs_last)/0.01; (y_in_lcs-y_in_lcs_last)/0.01];%(ts_in_lcs - ts_in_lcs_last);
        veh_vel = [0;0];
        veh_vel(1) = (x_in_utm-x_in_utm_last_meas)*20;%(x_in_lcs-x_in_lcs_last)*100;
        veh_vel(2) = (y_in_utm-y_in_utm_last_meas)*20;%(y_in_lcs-y_in_lcs_last)*100;%(ts_in_lcs - ts_in_lcs_last);

        x_in_utm_last_meas = x_in_utm;
        y_in_utm_last_meas = y_in_utm;

        x_in_utm_last = x_in_utm;
        y_in_utm_last = y_in_utm;
        a_in_utm_last = a_in_utm;
        t_in_utm_last = t_in_utm;
        veh_vel_last = veh_vel;
        wgs_ts_last = wgs_ts_in_sec;
    end
end


x_last = x_in_utm_last;
y_last = y_in_utm_last;
v_last = veh_vel_last;
veh_ori_pos = [veh_origin_pose.x_in_utm;veh_origin_pose.y_in_utm;veh_origin_pose.a_in_utm;veh_origin_pose.t_in_utm];

veh_pose = [x_in_lcs y_in_lcs a_in_lcs]';% x, y, yaw in LCS
vehicle_state = [veh_pose; veh_vel];
%}

if isempty(step)
    step = 0;
    state_flag = 0;
    configuration = struct('ellipses',true,'tags',false,'odometry',true, ...
                            'noise',true,'alpha',0.99,'step_by_step',false,...
                            'people',false,'proximity',50, 'ground',1,'map',2,'observations',3,...
                            'compatibility',4,'ground_hypothesis',5,'hypothesis',6,...
                            'tables',7);
    configuration.step_by_step = true;                
    veh_origin_pose = struct('x_in_utm',0,'y_in_utm',0,'a_in_utm',0,'t_in_utm',0);   

    veh_origin_pose.x_in_utm = x_in_utm;% in Mercator
    veh_origin_pose.y_in_utm = y_in_utm;% in Mercator
    veh_origin_pose.a_in_utm = a_in_utm;
    veh_origin_pose.t_in_utm = t_in_utm;
    
    %Square One
    x_in_lcs = 0;
    y_in_lcs = 0;
    a_in_lcs = 0;
%   t_in_lcs = 0;
    veh_pose = [0 0 0]';% x, y, yaw in LCS
    %standing still
    veh_vel = [0 0]';% V_x, V_y in LCS
    vehicle_state = [veh_pose; veh_vel];
    
    x_in_utm_last = x_in_utm;
    y_in_utm_last = y_in_utm;
    a_in_utm_last = a_in_utm;
    t_in_utm_last = t_in_utm;
    veh_vel_last = [0 0]';
    wgs_ts_last = wgs_ts_in_sec;
    x_in_utm_last_meas = x_in_utm;
    y_in_utm_last_meas = y_in_utm;
    
    innerLine_lcs = innerLine;
    middleLine_lcs = middleLine;
    outerLine_lcs = outerLine;
    landmarks_lcs = landmarks;
    patac_navi_lcs = patac_navi;
    object_list_lcs = object_list;
    object_num_lcs = object_num;
    object_list_update_lcs = object_list;
    CIPO_id_lcs = zeros(10,1);
    CIPO_id_next_lcs = zeros(10,1);
    sensor_data_raw_lcs = sensor_data_raw;
    ego_location_lcs = -1;
    cubic_poly_coef_lcs = zeros(1,4);
    
    landmarks_in_proximity_id_in_front = zeros(size(landmarks,1),1);
    landmarks_in_proximity_id_in_rear  = zeros(size(landmarks,1),1); 
    landmarks_in_proximity_id_in_rear_lcs  = landmarks_in_proximity_id_in_front;
    landmarks_in_proximity_id_in_front_lcs = landmarks_in_proximity_id_in_rear;
    
    maneuvers = patac_navi_lcs;
    global_landmarks = landmarks;
    odo_state_v = odo_motion_v;
    object_list_update = object_list_update_lcs;
    CIPO_id = CIPO_id_lcs;%zeros(10,1);
    CIPO_next_id = CIPO_id_next_lcs;%zeros(10,1);
    ego_location = -1; % due to invalid data from rtk ego locatoin cannot be determined.
   
    x_last = 0;
    y_last = 0;
    v_last = [0;0];
    delta_t = 0;
    veh_ori_pos = [veh_origin_pose.x_in_utm;veh_origin_pose.y_in_utm;veh_origin_pose.a_in_utm;veh_origin_pose.t_in_utm];
    step_out = step;

    innerLaneMark = innerLine_lcs.coordinate;
    middleLaneMark = middleLine_lcs.coordinate(:,1:2);
    outerLaneMark = outerLine_lcs.coordinate;    
    cubic_poly_coef = zeros(1,4);
    return;    
end
%after first cycle  
if( step ==0 )%initialisation
    if(abs(rtk_gps_lat)<1e-6 ...
       || abs(rtk_gps_lon)<1e-6 ...
       || abs(rtk_gps_ts)<1e-6...
      )
        % invalid rtk after first cycle
        step = 0;
        state_flag = 1;
        veh_pose = [0 0 0]';% x, y, yaw in LCS
        %standing still
        veh_vel = [0 0]';% V_x, V_y in LCS
        vehicle_state = [veh_pose; veh_vel];

        landmarks_in_proximity_id_in_front = zeros(size(landmarks,1),1);
        landmarks_in_proximity_id_in_rear  = zeros(size(landmarks,1),1); 
        maneuvers = patac_navi_lcs;
        global_landmarks = landmarks;
        odo_state_v = odo_motion_v;
        object_list_update = object_list_update_lcs;
        CIPO_id = zeros(10,1);
        CIPO_next_id = zeros(10,1);
        ego_location = -1;% due to invalid data from rtk ego locatoin cannot be determined.
        innerLaneMark = innerLine_lcs.coordinate;
        middleLaneMark = middleLine_lcs.coordinate(:,1:2);
        outerLaneMark = outerLine_lcs.coordinate;    
        step_out = step;
        cubic_poly_coef = zeros(1,4);

            delta_t = 0;
            x_last = x_in_utm_last;
            y_last = y_in_utm_last;
            v_last = veh_vel_last;
            veh_ori_pos = [veh_origin_pose.x_in_utm;veh_origin_pose.y_in_utm;veh_origin_pose.a_in_utm;veh_origin_pose.t_in_utm];
        return;
    end
end
 % valid rtk after first cycle for initialization, initialization most probably in this branch
    step = step+1;
    state_flag = 2;
if(step==1)
    veh_origin_pose.x_in_utm = x_in_utm;% in Mercator
    veh_origin_pose.y_in_utm = y_in_utm;% in Mercator
    veh_origin_pose.a_in_utm = a_in_utm;
    veh_origin_pose.t_in_utm = t_in_utm;

    x_in_lcs = 0;
    y_in_lcs = 0;
    a_in_lcs = a_in_utm;
%           t_in_lcs = wgs_ts_in_sec - veh_origin_pose.t_in_utm;
    veh_vel =  [0 0]';%[x_in_lcs-x_in_lcs_last; y_in_lcs-y_in_lcs_last]/(ts_in_lcs - ts_in_lcs_last);
    veh_pose = [x_in_lcs y_in_lcs a_in_lcs]';% x, y, yaw in LCS
    vehicle_state = [veh_pose; veh_vel];
%             delta_t = 0;
%             x_last = x_in_utm_last;
%             y_last = y_in_utm_last;
%             v_last = veh_vel_last;
%             veh_ori_pos = [veh_origin_pose.x_in_utm;veh_origin_pose.y_in_utm;veh_origin_pose.a_in_utm;veh_origin_pose.t_in_utm];

    x_in_utm_last = x_in_utm;
    y_in_utm_last = y_in_utm;
    a_in_utm_last = a_in_utm;
    t_in_utm_last = t_in_utm;
    veh_vel_last = [0 0]';  
    wgs_ts_last = wgs_ts_in_sec;
    x_in_utm_last_meas = x_in_utm;
    y_in_utm_last_meas = y_in_utm;


    innerLine_lcs.coordinate = innerLine.coordinate - ...
     [ones(size(innerLine.coordinate,1),1).*veh_origin_pose.x_in_utm,...
      ones(size(innerLine.coordinate,1),1).*veh_origin_pose.y_in_utm];
    middleLine_lcs.coordinate(:,1:2) = middleLine.coordinate(:,1:2) - ...
     [ones(size(middleLine.coordinate,1),1).*veh_origin_pose.x_in_utm,...
      ones(size(middleLine.coordinate,1),1).*veh_origin_pose.y_in_utm];
    outerLine_lcs.coordinate = outerLine.coordinate - ...
     [ones(size(outerLine.coordinate,1),1).*veh_origin_pose.x_in_utm,...
      ones(size(outerLine.coordinate,1),1).*veh_origin_pose.y_in_utm];

    %relocate landmarks in LCS, origin is the starting point.
    landmarks_lcs(:,1) = landmarks(:,1) - veh_origin_pose.x_in_utm;% landmarks in LCS!
    landmarks_lcs(:,2) = landmarks(:,2) - veh_origin_pose.y_in_utm;% landmarks in LCS!
    landmarks_lcs(:,3) = landmarks(:,3) - veh_origin_pose.x_in_utm;% landmarks in LCS!
    landmarks_lcs(:,4) = landmarks(:,4) - veh_origin_pose.y_in_utm;% landmarks in LCS!

    %relocate navigation maneuver list in LCS, origin is the starting point.
    patac_navi_lcs(:,1) = patac_navi(:,1) - veh_origin_pose.x_in_utm;%navigation maneuver in LCS!
    patac_navi_lcs(:,2) = patac_navi(:,2) - veh_origin_pose.y_in_utm;%navigation maneuver in LCS!

%         landmarks_in_proximity_id_in_front = zeros(size(landmarks,1),1);
%         landmarks_in_proximity_id_in_rear  = zeros(size(landmarks,1),1); 
%         object_list_update = object_list;
%         object_of_interest_id = 0;
%         ego_location = -1; % due to invalid data from rtk ego locatoin cannot be determined.
% 
%         maneuvers = patac_navi;
%          global_landmarks = landmarks_lcs;
%         odo_state_v = odo_motion_v;
%         step_out = step;
%         delta_t = 0;
%         innerLaneMark = innerLine_lcs.coordinate;
%         middleLaneMark = middleLine_lcs.coordinate;
%         outerLaneMark = outerLine_lcs.coordinate;    
%         x_last = x_in_utm_last;
%         y_last = y_in_utm_last;
%         v_last = veh_vel_last;
%         veh_ori_pos = [veh_origin_pose.x_in_utm;veh_origin_pose.y_in_utm;veh_origin_pose.a_in_utm;veh_origin_pose.t_in_utm];
    draw_map_nav_ground(innerLine_lcs.coordinate,innerLine_lcs.Vertex_index,...
                      middleLine_lcs.coordinate(:,1:2),middleLine_lcs.Vertex_index,...
                      outerLine_lcs.coordinate,outerLine_lcs.Vertex_index,...
                      landmarks_lcs, vehicle_state, patac_navi_lcs, configuration);        

end
% after initialization

state_flag = 3;

if(abs(rtk_gps_lat)<1e-6 ...
   || abs(rtk_gps_lon)<1e-6 ...
   || abs(rtk_gps_ts)<1e-6...
   )%if invalid rtk data after initialization halfways due to rtk signal loss
%             t_in_lcs = (t_in_utm_last +0.01)-veh_origin_pose.t_in_utm;
    x_in_lcs = (x_in_utm_last+veh_vel_last(1)*0.01)-veh_origin_pose.x_in_utm;
    y_in_lcs = (y_in_utm_last+veh_vel_last(2)*0.01)-veh_origin_pose.y_in_utm;
    a_in_lcs = a_in_utm_last;
    veh_vel = veh_vel_last; 

    x_in_utm_last = x_in_utm_last+veh_vel_last(1)*0.01;
    y_in_utm_last = y_in_utm_last+veh_vel_last(2)*0.01;
%   a_in_utm_last = a_in_utm_last;
    t_in_utm_last = t_in_utm+0.01;
    veh_vel_last = veh_vel;
    wgs_ts_last = wgs_ts_in_sec;
%         x_in_utm_last_meas = x_in_utm;
%         y_in_utm_last_meas = y_in_utm;            
else
    if(abs(wgs_ts_in_sec - wgs_ts_last)<1e-6)
%         t_in_lcs = (t_in_utm_last +0.01)-veh_origin_pose.t_in_utm;
        x_in_lcs = (x_in_utm_last+veh_vel_last(1)*0.01)-veh_origin_pose.x_in_utm;
        y_in_lcs = (y_in_utm_last+veh_vel_last(2)*0.01)-veh_origin_pose.y_in_utm;
        a_in_lcs = a_in_utm_last;
        veh_vel = veh_vel_last; 
        x_in_utm_last = x_in_utm_last+veh_vel_last(1)*0.01;
        y_in_utm_last = y_in_utm_last+veh_vel_last(2)*0.01;
%               a_in_utm_last = a_in_utm_last;
        t_in_utm_last = t_in_utm+0.01;
%               veh_vel_last = veh_vel;
%               wgs_ts_last = wgs_ts_in_sec;
%               x_in_utm_last_meas = x_in_utm;
%               y_in_utm_last_meas = y_in_utm;
    else
%               t_in_lcs = t_in_utm - veh_origin_pose.t_in_utm;
        x_in_lcs = x_in_utm-veh_origin_pose.x_in_utm;
        y_in_lcs = y_in_utm-veh_origin_pose.y_in_utm;
        a_in_lcs = a_in_utm; %a_in_utm - veh_origin_pose.a_in_utm;
%                 veh_vel = [(x_in_utm-x_in_utm_last)/0.01; (y_in_utm-y_in_utm_last)/0.01];%(ts_in_lcs - ts_in_lcs_last);
        veh_vel = [(x_in_utm-x_in_utm_last); (y_in_utm-y_in_utm_last)]/(t_in_utm - t_in_utm_last);
%                 veh_vel = [0;0];
%                 veh_vel(1) = (x_in_utm-x_in_utm_last_meas)*20;%(x_in_lcs-x_in_lcs_last)*100;
%                 veh_vel(2) = (y_in_utm-y_in_utm_last_meas)*20;%(y_in_lcs-y_in_lcs_last)*100;%(ts_in_lcs - ts_in_lcs_last);

        x_in_utm_last_meas = x_in_utm;
        y_in_utm_last_meas = y_in_utm;

        x_in_utm_last = x_in_utm;
        y_in_utm_last = y_in_utm;
        a_in_utm_last = a_in_utm;
        t_in_utm_last = t_in_utm;
        veh_vel_last = veh_vel;
        wgs_ts_last = wgs_ts_in_sec;               
    end
end

delta_t = (t_in_utm - t_in_utm_last);
x_last = x_in_utm_last;
y_last = y_in_utm_last;
v_last = veh_vel_last;
veh_ori_pos = [veh_origin_pose.x_in_utm;veh_origin_pose.y_in_utm;veh_origin_pose.a_in_utm;veh_origin_pose.t_in_utm];

veh_pose = [x_in_lcs y_in_lcs a_in_lcs]';% x, y, yaw in LCS
vehicle_state = [veh_pose; veh_vel];

if(mod(step,2)==1)
    %Relocate object list in LCS
    %SCS --> VCS offset 
    x_scs_offset = 5.24/2.0;%vehicle length 5.24m
    y_scs_offset = 0;%vehicle width 1.8m

    %VCS --> CCS (Control Coordinate System) offset 
    x_ccs_offset = 0;
    y_ccs_offset = 0;
    % x_offset = x_scs_offset+x_ccs_offset;
    % y_offset = y_scs_offset+y_ccs_offset;
	object_list_id = object_list(:,1);
	valid = find(object_list_id);
	object_num_meas = size(valid,1);

    object_list_lcs = object_list;
    object_num_lcs = object_num_meas;

    object_loc = [object_list(1:object_num_meas,2),object_list(1:object_num_meas,4)]';
    object_loc(1,:) = object_loc(1,:) + x_scs_offset;
    object_loc(2,:) = object_loc(2,:) + y_scs_offset;
    Tab = [ x_in_lcs; y_in_lcs; a_in_lcs];
    Pa = tpcomp(Tab, object_loc);
    object_list_lcs(1:object_num_meas,2)=Pa(1,:)';%object_list(:,2)+ x_scs_offset + x_in_lcs;%RangeX in LCS, displacement of VCS to SCS by 2.6m, half of vehicle length
    object_list_lcs(1:object_num_meas,4)=Pa(2,:)';%object_list(:,4)+ y_scs_offset + y_in_lcs;%RangeY in LCS

    object_vec = [object_list(1:object_num_meas,3),object_list(1:object_num_meas,5)]';
    Tab = [ veh_vel(1); veh_vel(2); a_in_lcs];
    Pb = tpcomp(Tab, object_vec);
    object_list_lcs(1:object_num_meas,3)=Pb(1,:)';%object_list(:,3)+ veh_vel(1);%VelX
    object_list_lcs(1:object_num_meas,5)=Pb(2,:)';%object_list(:,5)+ veh_vel(2);%VelY    

   raw_loc = [sensor_data_raw(:,2),sensor_data_raw(:,3)]';
   Tab = [ x_scs_offset + x_in_lcs; y_scs_offset + y_in_lcs; a_in_lcs];
   Pa = tpcomp(Tab, raw_loc);
   sensor_data_raw_lcs = sensor_data_raw;
   sensor_data_raw_lcs(:,2)=Pa(1,:)';%object_list(:,2)+ x_scs_offset + x_in_lcs;%RangeX in LCS, displacement of VCS to SCS by 2.6m, half of vehicle length
   sensor_data_raw_lcs(:,3)=Pa(2,:)';%object_list(:,4)+ y_scs_offset + y_in_lcs;%RangeY in LCS

    %find local landmarks
    option.firstTime=1;
    option.clockWise=0;  %0 for counter-clockwise，1 for clockwise    
    [landmarks_in_proximity_id_in_front_lcs, landmarks_in_proximity_id_in_rear_lcs] = ...
        quest_map4landmark_new_lane (x_in_lcs, y_in_lcs, landmarks_lcs,middleLine_lcs.coordinate(:,1:2), middleLine_lcs.Vertex_index,configuration,option);

%     landmarks_in_proximity_id_in_front = landmarks_in_proximity_id_in_front_lcs;
%     landmarks_in_proximity_id_in_rear = landmarks_in_proximity_id_in_rear_lcs;
    %     sensor_data_raw(:,2)=sensor_data_raw(:,2)+ x0_offset + veh_origin_pose.x;%RangeX, displacement of VCS to SCS by 2.6m, half of vehicle length
    %     sensor_data_raw(:,3)=sensor_data_raw(:,3)+ y0_offset + veh_origin_pose.y;%RangeY

    %    [veh_pose, veh_vel] = slam_ekf_patac (inner, middle, outer, landmarks, ...
    %         odo_motion_x, odo_motion_y, odo_motion_yaw, ...

    %         rtk_gps_lat, rtk_gps_lon, rtk_gps_yaw, rtk_gps_ts, ...
    %         sensor_data_raw,...
    %         proximity);

    
    
end

if(mod(step,2)==0)
    %locate objects in lanes    
    %decide object list location in lanes
    option.firstTime=1;
    option.clockWise=0;  %0 for counter-clockwise，1 for clockwise
    object_list_update_lcs  = object_localization(object_list_lcs, object_num_lcs, innerLine_lcs.coordinate,innerLine_lcs.Vertex_index,...
                                                          middleLine_lcs.coordinate(:,1:2),middleLine_lcs.Vertex_index,...
                                                          outerLine_lcs.coordinate,outerLine_lcs.Vertex_index,...
                                                          lane_offset,...
                                                          option);
end

if(mod(step,2)==1)
    %find the closest object in ego lane (probably CIPO)
    option.firstTime=1;
    option.clockWise=0;  %0 for counter-clockwise，1 for clockwise    
    [obj_closest_in_path_ID, obj_closest_in_next_path_ID,ego_location_lcs] = objects_of_interest_fast(object_list_update_lcs,object_num_lcs,...
            x_in_lcs, y_in_lcs, innerLine_lcs.coordinate, innerLine_lcs.Vertex_index,...
            middleLine_lcs.coordinate(:,1:2), middleLine_lcs.Vertex_index,...
            outerLine_lcs.coordinate, outerLine_lcs.Vertex_index, option);
    obj_in_path_size = size(obj_closest_in_path_ID,1);
    if(~isempty(obj_closest_in_path_ID))
        CIPO_id_lcs = zeros(10,1);
        if(obj_in_path_size<=10)
            CIPO_id_lcs(1:obj_in_path_size,1) = obj_closest_in_path_ID(1:obj_in_path_size,1);
        else
            CIPO_id_lcs(1:10,1) = obj_closest_in_path_ID(1:10,1);
        end
    end
    obj_in_path_size = size(obj_closest_in_next_path_ID,1);
    if(~isempty(obj_closest_in_next_path_ID))
        CIPO_id_next_lcs = zeros(10,1);
        if(obj_in_path_size<=10)
            CIPO_id_next_lcs(1:obj_in_path_size) = obj_closest_in_next_path_ID(1:obj_in_path_size,1);
        else
            CIPO_id_next_lcs(1:10,1) = obj_closest_in_next_path_ID(1:10);
        end
    end
    point_num = 30;
    right_shift = 2; %counter-clockwise right shift 2m for outer lane; left shift for inner lane.
    cubic_poly_coef_lcs = lanefit(vehicle_state, option.clockWise, middleLine_lcs, outerLine_lcs, right_shift, ego_location_lcs, point_num);
end


draw_observations_nav_ground(innerLine_lcs.coordinate, innerLine_lcs.Vertex_index,...
                                middleLine_lcs.coordinate(:,1:2), middleLine_lcs.Vertex_index,...
                                outerLine_lcs.coordinate, outerLine_lcs.Vertex_index,...
                                landmarks_lcs, landmarks_in_proximity_id_in_front_lcs, ...
                                landmarks_in_proximity_id_in_rear_lcs, vehicle_state, object_list_lcs, ...
                                CIPO_id_lcs, CIPO_id_next_lcs, sensor_data_raw_lcs, ...
                                patac_navi_lcs,configuration);
maneuvers = patac_navi_lcs;
odo_state_v = odo_motion_v;
global_landmarks = landmarks_lcs;
landmarks_in_proximity_id_in_front = landmarks_in_proximity_id_in_front_lcs;
landmarks_in_proximity_id_in_rear = landmarks_in_proximity_id_in_rear_lcs;
object_list_update = object_list_update_lcs;
CIPO_id = CIPO_id_lcs;
CIPO_next_id = CIPO_id_next_lcs;
cubic_poly_coef = cubic_poly_coef_lcs;

ego_location = ego_location_lcs;
innerLaneMark = innerLine_lcs.coordinate;
middleLaneMark = middleLine_lcs.coordinate(:,1:2);
outerLaneMark = outerLine_lcs.coordinate;
step_out = step;
