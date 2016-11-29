

function map = EKF_prediction (map, motion)
%-------------------------------------------------------
% University of Zaragoza
% Centro Politecnico Superior
% Robotics and Real Time Group
% Authors:  J. Neira, J. Tardos
% Date   :  7-2004
%-------------------------------------------------------
%-------------------------------------------------------
%global configuration;
coder.extrinsic('sparse');

twr = map.x(1:3);
trn = motion.x;

j1 = jacobian1(twr, trn);
j2 = jacobian2(twr, trn);

J1 = sparse(blkdiag(j1, eye(2 * map.n)));%F_k
J2 = sparse([j2;  zeros(2 * map.n, 3)]);%G_k
map.x(1:3) = tcomp (twr, trn);%<15>
map.P = J1 * map.P * J1' + J2 * motion.P * J2';%<16>

tab = map.odometry(map.n).x;
Pab = map.odometry(map.n).P;
tbc = trn;
Pbc = motion.P;
[tac, Pac] = ucomp(tab, Pab, tbc, Pbc);
map.odometry(map.n+1).x = tac;
map.odometry(map.n+1).P = Pac;



