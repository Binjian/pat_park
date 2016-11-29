
function [veh_pose, veh_vel] = slam_ekf_patac (innerLine, middleLine, outerLine, landmarks, ...
                   odo_motion_x, odo_motion_y, odo_motion_yaw, ...
                   rtk_gps_lat, rtk_gps_lon, rtk_gps_yaw, rtk_gps_ts,...
                   sensor_data_raw,...
                   proximity)
coder.extrinsic('EKF_prediction');    
coder.extrinsic('draw_ellipse');    
rng;%randn('state', 0);

% determines execution and display modes
coder.inline('never');
%global configuration sensor;

persistent sensor configuration step veh_origin_pose map ground; %step = 0;
%chi2 = chi2inv(configuration.alpha,1:1000);
persistent rtk_gps_lat_last rtk_gps_lon_last rtk_gps_ts_last;

%local_map = zeros(1,10);
%vehicle_state = ones(3,1);

if isempty(step)
    %adapt to applied sensors (SRR)!
    step = 1;

configuration = struct('ellipses',true,'tags',false,'odometry',true, ...
                        'noise',true,'alpha',0.99,'step_by_step',false,...
                        'people',false,'ground',1,'map',2,'observations',3,...
                        'compatibility',4,'ground_hypothesis',5,'hypothesis',6,...
                        'tables',7);
                    
%     configuration.ellipses = true;
%     configuration.tags = false;
%     configuration.odometry = true;
%     configuration.noise = true;
%     configuration.alpha = 0.99;
%     configuration.step_by_step = true;
%     configuration.people = true;
% 
%     % figure numbers
%     configuration.ground = uint32(1);
%     configuration.map = uint32(2);
%     configuration.observations = uint32(3);
%     configuration.compatibility = uint32(4);
%     configuration.ground_hypothesis = uint32(5);
%     configuration.hypothesis = uint32(6);
%     configuration.tables = uint32(7);



    sensor.range = 5;
    sensor.minangle = -pi/2;
    sensor.maxangle = pi/2;
    sensor.srho = 0.01;
    sensor.stita = 0.125*pi/180;

    rtk_gps_lat_last =rtk_gps_lat;
    rtk_gps_lon_last = rtk_gps_lon;
    rtk_gps_yaw_last = rtk_gps_yaw;
    rtk_gps_ts_last = rtk_gps_ts;     
    % generate the ground data from hdmap and RTK
    %landmarks.x landmarks.y (= hd_map.landmarks(:,2);)
    ground = generate_rtk_ground(innerLine, middleLine, outerLine);

    % start with a fresh map
    [map, ground] = new_map(ground);

    % plot ground
    draw_ground(ground,landmarks, configuration);
    %%pause

    % ok, here we go

    %%observations = get_observations(ground, sensor, step);
    [x1,y1,utmzone,utmhemi] = wgs2utm(rtk_gps_lat,rtk_gps_lon,51,'N');
    veh_origin_pose.x = x1;
    veh_origin_pose.y = y1;
    veh_origin_pose.yaw = rtk_gps_yaw;
    veh_origin_pose.ts = rtk_gps_ts;
    veh_pose = veh_origin_pose;
    veh_vel = [0; 0]; % Start point with 0 velocity.
    observations = get_observations (ground, landmarks, veh_pose, sensor_data_raw, sensor, proximity);
    draw_observations (observations, configuration, step);
    
%     GT = zeros(1, size(sensor_data_raw,1));
%      H = zeros(1, size(sensor_data_raw,1));
    
    map = add_features(map, observations);


    %results_in.total = [];
    %results_in.true.positives = [];
    %results_in.true.negatives = [];
    %results_in.false.positives = [];
    %results_in.false.negatives = [];

    %results = store_results (results_in, observations, GT, H);

    % plot map
%     configuration.name = '';
    draw_map (map, ground,configuration,step);

   % steps = length(ground.motion);
else

    step = step+1;
    disp('--------------------------------------------------------------');
%     disp(sprintf('Step: %d', step));
    
    % EKF prediction step
    
    %odometry = get_odometry (motion);
    %odometry.x = [0 0 0]';
    %odo_motion.P = diag([0.25 0.1 5*pi/180].^2);
    odometry.x = [odo_motion_x, odo_motion_y,odo_motion_yaw]'; ;%odo_motion.x;
    odometry.P = diag([0.25 0.1 5*pi/180].^2);

   
    map = EKF_prediction (map, odometry);    

    % sense
    [x1,y1,utmzone,utmhemi] = wgs2utm(rtk_gps_lat,rtk_gps_lon,51,'N');
    veh_pose.x = x1;
    veh_pose.y = y1;
    veh_pose.yaw = rtk_gps_yaw;
    veh_pose.ts = rtk_gps_ts;

    [x2,y2,utmzone,utmhemi] = wgs2utm(rtk_gps_lat_last,rtk_gps_lon_last,51,'N');
    veh_vel = [x1-x2; y1-y2]/(rtk_gps_ts - rtk_gps_ts_last)/1000;%ms-->s

    rtk_gps_lat_last =rtk_gps_lat;
    rtk_gps_lon_last = rtk_gps_lon;
%     rtk_gps_yaw_last = rtk_gps_yaw;
    rtk_gps_ts_last = rtk_gps_ts; 


    motion.x = [x1 y1 rtk_gps_yaw]';
    motion.P = diag([0.02 0.02 2*pi/180].^2); % expectation of std of RTK   
    ground = move_vehicle (ground, motion, step);    

    observations = get_observations(ground, landmarks, veh_pose, sensor_data_raw, sensor, proximity);
    
    % individual compatibility
    prediction = predict_observations (map, ground);
    compatibility = compute_compatibility (prediction, observations);

%     disp(sprintf('Hypothesis: %d', compatibility.HS));
%     disp(['IC: ' sprintf('%2d   ', compatibility.AL)]);
    disp(compatibility.HS);
    disp(compatibility.AL);

    disp(' ');
    
    % ground truth
 %   GT = ground_solution(map, observations);
 %   disp(['GT: ' sprintf('%2d   ', GT)]);
 %   disp(' ');

    % your algorithm here!
    % 1. Try NN
    % 2. Complete SINGLES and try it
    % 3. Include people and try SINGLES5
    % 4. Try JCBB
    
    H = NN (prediction, observations, compatibility,configuration);
    %H = JCBB (prediction, observations, compatibility, configuration);
%    configuration.step_by_step = not(prod(H == GT)); % discrepance with ground truth

    draw_map (map, ground,configuration, step);
    draw_observations (observations, configuration, step);
    
    
    draw_compatibility (prediction, observations, compatibility,configuration);

%     disp(['H : ' sprintf('%2d   ', H)]);
%     disp(['    ' sprintf('%2d   ', GT == H)]);
    disp(' ');
    
    draw_hypothesis (prediction, observations, H, 'NN:', 'b-',configuration);
%     draw_hypothesis (prediction, observations, GT, 'JCBB:', 'b-',configuration);
%     draw_tables (compatibility, GT, H, configuration);
    
    % update EKF step
    map = EKF_update (map, prediction, observations, H, step);
    

    % only new features with no neighbours
    new = find((H == 0) & (compatibility.AL == 0));
    
    if nnz(new)
%        disp(['NEW: ' sprintf('%2d   ', new)]);
       map = add_features(map, observations, new);
    end

    draw_map (map, ground, configuration, step);
    %results_in = results;
    %results = store_results(results_in, observations, [], H);
    
%     if configuration.step_by_step
%         wait
%     else
%         drawnow;
%     end
    
end
veh_pose = map.x(1:3);
%local_map = map;

%%show_results(map, ground, results);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%}
