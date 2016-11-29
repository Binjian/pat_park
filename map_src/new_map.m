

function [map, ground] = new_map(ground)
%-------------------------------------------------------
% University of Zaragoza
% Centro Politecnico Superior
% Robotics and Real Time Group
% Authors:  J. Neira, J. Tardos
% Date   :  7-2004
%-------------------------------------------------------
%-------------------------------------------------------
trajectory_struct = struct('x',[0 0 0]','P',zeros(3,3));
trajectory_array = repmat(trajectory_struct,1,1000);

%%Maximal 1000 feature points!
map = struct('n',0,'x',[zeros(1,3), zeros(1,1000*2)]','P',zeros(3+2*1000,3+2*1000),'ground_id',zeros(1,1000),...
             'estimated',trajectory_array,'odometry',trajectory_array);

% map.n = 0;
% map.x = [0 0 0]';
% map.P = zeros(3,3);
% map.ground_id = [];
% map.estimated(1).x = [0 0 0]';
% map.estimated(1).P = map.P;
% map.odometry(1).x = [0 0 0]';
% map.odometry(1).P = map.P;

ground.trajectory(1).x = [0 0 0]';
ground.trajectory(1).P = zeros(3, 3);

