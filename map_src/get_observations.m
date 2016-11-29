function observations = get_observations (ground, landmarks, veh_pose, sensor_data_raw, sensor, proximity)
%-------------------------------------------------------
% University of Zaragoza
% Centro Politecnico Superior
% Robotics and Real Time Group
% Authors:  J. Neira, J. Tardos
% Date   :  7-2004
% ground and veh_pose for setting observation.ground
%-------------------------------------------------------
%-------------------------------------------------------
% global sensor;
roi.x=proximity;
roi.y=proximity;

%xp = landmarks.x;lanemarking
%yp = landmarks.y;
%    people.x = [people.x; xp']; people.y = [people.y; yp'];

%Landmarks
x1 = double(landmarks(1,:)) - ones(1, size(landmarks,2))*veh_pose.x;%landmarks row vec
y1 = double(landmarks(2,:)) - ones(1, size(landmarks,2))*veh_pose.y;
%visible = find( (abs(x1) <= roi.x) & (abs(y1) <= roi.y) );
visible = find( (x1.^2 + y1.^2) <= proximity*proximity);
observations.local_landmarks_id = visible;
observations.local_landmarks = landmarks(visible,:);


%Inner ring
x1 = double(ground.inner(1,:)) - ones(1,size(ground.inner,2))*veh_pose.x;
y1 = double(ground.inner(2,:)) - ones(1,size(ground.inner,2))*veh_pose.y;
visible = find( (abs(x1) <= roi.x) & (abs(y1) <= roi.y) );
observations.local_ground.inner = ground.inner(visible,:);
observations.local_ground.inner_id = visible;
%Middle ring
x1 = double(ground.middle(1,:)) - ones(1,size(ground.middle,2))*veh_pose.x;
y1 = double(ground.middle(2,:)) - ones(1,size(ground.middle,2))*veh_pose.y;
visible = find( (abs(x1) <= roi.x) & (abs(y1) <= roi.y) );
observations.local_ground.middle= ground.middle(visible,:);
observations.local_ground.middle_id = visible;
%Outer ring
x1 = double(ground.outer(1,:)) - ones(1,size(ground.outer,2))*veh_pose.x;
y1 = double(ground.outer(2,:)) - ones(1,size(ground.outer,2))*veh_pose.y;
visible = find( (abs(x1) <= roi.x) & (abs(y1) <= roi.y) );
observations.local_ground.outer = ground.outer(visible,:);
observations.local_ground.outer_id = visible;


validity = find(sensor_data_raw(:,1));
sensor_detection = sensor_data_raw(validity,:);
sensor_data_raw_size = length(validity);

%%%%%Apply Coordinate Transform from Sensor Coordinate System (middle of front bumper) to Vehicle Coordinat System (Centor of Mass)
%%%%%%
% x0_offset = -5.24/2.0;%vehicle length 5.24m
% y0_offset = 0;%vehicle width 1.8m
% xn = sensor_detection(:,2)+x0_offset;
% yn = sensor_detection(:,3)+y0_offset;
%%%%Code here!!!
%%%%%%%%%%%%%%
xn =sensor_detection(:,2);
yn =sensor_detection(:,3);

[titav, rhov] = cart2pol (xn, yn);

srho = rhov * sensor.srho;
stita = ones(size(titav)) * sensor.stita;
Rn = zeros(length(srho)*2);

for i=1:length(srho),
    Ri = diag([srho(i)^2 stita(i)^2]);
    Ji = [cos(titav(i)) -rhov(i)*sin(titav(i))
        sin(titav(i))  rhov(i)*cos(titav(i))];
    
    % rhoi = rhov(i);
    % titai = titav(i);
    % [xi, yi] = pol2cart(titai,rhoi);
    Ri = Ji * Ri * Ji';
    Rn(2*i-1:2*i,2*i-1:2*i) = Ri;
end

trp = [xn'; yn'];
trp = reshape(trp, [], 1);
observations.m = sensor_data_raw_size;
observations.z = trp;
observations.R = Rn;

%on_the_road = find_object_on_the_road(observations, hdmap);
%observations.on_the_road = on_the_road;

