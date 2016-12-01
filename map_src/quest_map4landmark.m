function landmarks_in_proximity_id = quest_map4landmark (x_in_lcs, y_in_lcs, landmarks, configuration)
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


landmarks_in_proximity_id = zeros(size(landmarks,1),1);
% landmarks_in_proximity = zeros(size(landmarks));
%Landmarks
x1 = double(landmarks(:,1)) - ones(size(landmarks,1),1)*x_in_lcs;%landmarks column vector
y1 = double(landmarks(:,2)) - ones(size(landmarks,1),1)*y_in_lcs;
%visible = find( (abs(x1) <= roi.x) & (abs(y1) <= roi.y) );
visible = find( (x1.^2 + y1.^2) <= configuration.proximity*configuration.proximity);
landmarks_in_proximity_id(1:size(visible,1)) = visible;
% landmarks_in_proximity(1:size(visible,1),:) = landmarks(visible,:);
%landmarks_in_proximity = visible;
