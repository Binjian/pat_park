clear all
clc

load middleLine
% load middle_line_all
% middleLine.coordinate=[middle_mercator_lat,middle_mercator_lon];
% middleLine.Vertex_index=vortex_index;
% save middleLine middleLine


middleLine.coordinate = middleLine.coordinate - ...
                        [ones(size(middleLine.coordinate,1),1)*middleLine.coordinate(1,1),ones(size(middleLine.coordinate,1),1).*middleLine.coordinate(1,2)];

x=middleLine.coordinate(:,1);
y=middleLine.coordinate(:,2);

figure(1)
plot(x,y,'b.'); hold on 


tic
option.firstTime=1;
option.clockWise=0;  %0是逆时针，1是顺时针
p=middleLine.coordinate(50,:)+[10,10];
[distance,flag,nearestIndex]=calDistance(p,middleLine,option)
toc
plot(p(1),p(2),'r*');

% tic
% option.first=0;
% option.oldIndex=nearestIndex;
% % p=middleLine.coordinate(end,:)+[10,10];
% [distance,flag,nearestIndex]=calDistance(p,middleLine,option)
% toc
% plot(p(1),p(2),'k*');