
function ground = move_vehicle (ground, motion, step)
%-------------------------------------------------------
% University of Zaragoza
% Centro Politecnico Superior
% Robotics and Real Time Group
% Authors:  J. Neira, J. Tardos
% Date   :  7-2004
%-------------------------------------------------------
%-------------------------------------------------------

ground.trajectory(step).x = tcomp(ground.trajectory(step-1).x, motion.x);
ground.trajectory(step).P = zeros(3, 3);


