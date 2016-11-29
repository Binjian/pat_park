function draw_trajectory (trajectory, color, configuration, step)
%-------------------------------------------------------
% University of Zaragoza
% Centro Politecnico Superior
% Robotics and Real Time Group
% Authors:  J. Neira, J. Tardos
% Date   :  7-2004
%-------------------------------------------------------
%-------------------------------------------------------
% global configuration;

%draw locations

if nargin == 4
	%loc = [trajectory(1:step).x];
	loc = zeros(step,3);
	for i = 1: step,		
		loc(i,:) = [trajectory(i).x];
	end 
else
	%loc = [trajectory(:).x];
	loc = zeros(length(trajectory),3);
	for i = 1: length(trajectory),
		loc(i,:) = [trajectory(i).x];
	end 
end


x = loc(1,:)';
y = loc(2,:)';
plot(x, y, [color '-']);
plot(x, y, [color '.']);

for p = 1:length(x),
    if configuration.ellipses
        draw_ellipse (trajectory(p).x, trajectory(p).P, color);
    end
end
draw_vehicle(trajectory(1).x, trajectory(1).P, color,configuration);
draw_vehicle(trajectory(end).x, trajectory(end).P, color,configuration);
