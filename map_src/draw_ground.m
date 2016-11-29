

function draw_ground (ground, landmarks, configuration)
%-------------------------------------------------------
% University of Zaragoza
% Centro Politecnico Superior
% Robotics and Real Time Group
% Authors:  J. Neira, J. Tardos
% Date   :  7-2004
%-------------------------------------------------------
%-------------------------------------------------------
%global configuration;

figure(configuration.ground); clf; axis equal; hold on;
plot(ground.inner(1,:), ground.inner(2,:), 'r-');
plot(ground.middle(1,:), ground.middle(2,:), 'y-');
plot(ground.outer(1,:), ground.outer(2,:), 'r-');

for i = 1: size (landmarks,1)
    if(landmarks(i,5)<=60)%Type 10,20, ...,60 is manuveur point
        %plot(observations.local_landmarks(i,1), observations.local_landmarks(i,2),'rd', 'MarkSize', 10);
        rectangle('Position',landmarks(i,1:4),'Curvature',0.2,...
                   'FaceColor',[0.5 0 0],'EdgeColor','r','LineWidth',5)
    else
        %plot(observations.local_landmarks(i,1), observations.local_landmarks(i,2),'rh', 'MarkSize', 10);
        rectangle('Position',landmarks(i,1:4),'Curvature',1,...
                   'FaceColor',[0.5 0 0.5],'EdgeColor','b','LineWidth',2)
    end
end

%{
trajectory(1).x = [0 0 0]';
trajectory(1).P = zeros(3, 3);
for step = 1 : length(ground.motion),
    [trajectory(step+1).x, trajectory(step+1).P] = ...
        ucomp(trajectory(step).x, trajectory(step).P, ...
        ground.motion(step).x, zeros(3,3));
end
draw_trajectory (trajectory, 'r');
%}
%title(sprintf('GROUND TRUTH, features: %d', ground.n));

