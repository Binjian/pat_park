%-------------------------------------------------------
% Patac
% SHD
% Software
%-------------------------------------------------------
% Authors:  Binjian Xin
% Date   :  2017-1
%
% calculates the inverse of one or more transformations
%-------------------------------------------------------
function [distance,flag]=calDistance_fast(p,coordinate, Vertex_index,option)
% 返回distance是垂直距离，flag是方向，1是右边，0是重合，-1左左边
% 输入p是当前车辆的坐标
% 输入offlineData是离线数据的结构体，（coordinate，Vertex_index）
%        offlineData.coordinate是地图坐标
%        offlineData.Vertex_index是每段数据对于得顶点
% option 是个结构体（firstTime,oldIndex,clockWise）
%        如果 option.firstTime==1，则不需要提供option.oldIndex
%        如果 option.firstTime==0，则需要提供option.oldIndex
%        option.clockWise=1是顺时针;  =0是逆时针，


%%
flag = false;
% coordinate = offlineData.coordinate;
% Vertex_index = offlineData.Vertex_index;
d_side = zeros(1,4);
for i=1:4                 %计算当前点在离线地图的哪个边上  sideIndex
    A=coordinate(Vertex_index(i,1),:);
    B=coordinate(Vertex_index(i,2),:);
   [d_side(i),flag]=point2line(p,A,B);
end
[~,sideIndex]=min(d_side);

% A=coordinate(Vertex_index(sideIndex,1),:);
% B=coordinate(Vertex_index(sideIndex,2),:);
% [d_side(i),flag]=point2line(p,A,B);

if option.clockWise %顺时针
    A_index=Vertex_index(sideIndex,2);
    B_index=Vertex_index(sideIndex,1);

else          %逆时针
    A_index=Vertex_index(sideIndex,1);
    B_index=Vertex_index(sideIndex,2);
end

[distance,flag]=point2line(p,coordinate(A_index,:),coordinate(B_index,:));

