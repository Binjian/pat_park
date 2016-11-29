
function ground = generate_rtk_ground(innerLine, middleLine, outerLine)
%-------------------------------------------------------
% University of Zaragoza
% Centro Politecnico Superior
% Robotics and Real Time Group
% Authors:  J. Neira, J. Tardos
% Date   :  7-2004
%-------------------------------------------------------
% Generate experiment
%-------------------------------------------------------
trajectory_struct = struct('x',[0 0 0]','P',zeros(3,3));
trajectory_array = repmat(trajectory_struct,1,2000);
ground = struct('inner',innerLine,'middle',middleLine,'outer',outerLine,'trajectory', trajectory_array);

ground.trajectory(1).x = [0 0 0]';
ground.trajectory(1).P = zeros(3, 3);


%landmarks.x = hd_map.landmarks(:,1);
%landmarks.y = hd_map.landmarks(:,2);
%landmarks.type = hd_map.landmarks(:,3);
%landmarks = landmarks;

