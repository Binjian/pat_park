function [distance,flag,nearestIndex]=calDistance(p,coordinate, Vertex_index,option)
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
a = 0;
nearestIndex = 0;
for i=1:4                 %计算当前点在离线地图的哪个边上  sideIndex
    A=coordinate(Vertex_index(i,1),:);
    B=coordinate(Vertex_index(i,2),:);
   [d_side(i),flag]=point2line(p,A,B);
end
[a,sideIndex]=min(d_side);




%%  首次计算
if option.firstTime==1
    d1 = zeros(1000,1);
%     d = zeros((Vertex_index(sideIndex,2) - Vertex_index(sideIndex,1)),1);
    for i=1:(Vertex_index(sideIndex,2) - Vertex_index(sideIndex,1))
        A=coordinate(Vertex_index(sideIndex,1)+i,:);
        d1(i)=squareDistance(p,A);
    end
    len = Vertex_index(sideIndex,2) - Vertex_index(sideIndex,1);
    d = d1(1:len);
    [a,nearestIndex]=min(d);  %计算点在边上的最近的点
    
    if nearestIndex==length(d);
        nearestIndex=nearestIndex-1;
    end
    if nearestIndex==1;
        nearestIndex=nearestIndex+1;
    end
    
    if d(nearestIndex+1)<d(nearestIndex-1)        
        if option.clockWise %顺时针
            
            B_index=nearestIndex+Vertex_index(sideIndex,1);
            A_index=B_index+1;
            if A_index>Vertex_index(sideIndex,2)
                A_index=A_index-1;
                B_index=B_index-1;
            end
            
        else              %逆时针
            
            A_index=nearestIndex+Vertex_index(sideIndex,1);
            B_index=A_index+1;
            if B_index>Vertex_index(sideIndex,2)
                A_index=A_index-1;
                B_index=B_index-1;
            end
        end 
        
    else
        
        if option.clockWise %顺时针
            A_index=nearestIndex+Vertex_index(sideIndex,1);
            B_index=A_index-1;
            if B_index<Vertex_index(sideIndex,1)
                A_index=A_index+1;
                B_index=B_index+1;
            end
        else          %逆时针
            B_index=nearestIndex+Vertex_index(sideIndex,1);
            A_index=B_index-1;
            if A_index<Vertex_index(sideIndex,1)
                A_index=A_index+1;
                B_index=B_index+1;
            end
        end
    end
else
    A_index = 0;
    B_index = 1;
end    
    

% else 
% %%  非首次计算
% 
% %     oldIndex=option.oldIndex;
%     oldIndex = 0;
%     temp = zeros(21,1);
%     d = temp;
%     if option.clockWise %%% 非首次 + 顺时针
%         for i=-15:5;
%             temp(i)=oldIndex+i;
%             if temp(i)>size(coordinate,1)
%                 temp(i)=mod(temp(i),size(coordinate,1));
%             end
%             if temp(i)<=0
%                 temp(i)=size(coordinate,1)-temp(i);
%             end
%             A=coordinate(temp,:);
%             d(i)=squareDistance(p,A);
%         end
%         [a,nearestIndex]=min(d);
%         
%         
%         if nearestIndex==length(d);
%            nearestIndex=nearestIndex-1;
%         end
%         if nearestIndex==1;
%             nearestIndex=nearestIndex+1;
%         end
%         
%         
%         if d(nearestIndex+1)<d(nearestIndex-1)
%             B_index=temp(nearestIndex);
%             A_index=B_index+1;
%             if A_index>Vertex_index(sideIndex,2)
%                 A_index=A_index-1;
%                 B_index=B_index-1;
%             end
%         else
%             A_index=temp(nearestIndex);
%             B_index=A_index-1;
%             if B_index<Vertex_index(sideIndex,1)
%                 A_index=A_index+1;
%                 B_index=B_index+1;
%             end
%         end       
%       
%         
%     else   %% 非首次 + 逆时针
%         
%         
%         for i=-5:15;
%             temp(i)=oldIndex+i;
%             if temp(i)>size(coordinate,1)
%                 temp(i)=mod(temp(i),size(coordinate,1));
%             end
%             if temp(i)<=0
%                 temp(i)=size(coordinate,1)-temp(i);
%             end
%             A=coordinate(temp,:);
%             d(i)=squareDistance(p,A);
%         end
%         [a,nearestIndex]=min(d);
%         
%         if d(nearestIndex+1)<d(nearestIndex-1)
%             A_index=temp(nearestIndex);
%             B_index=A_index+1;
%             if B_index>Vertex_index(sideIndex,2)
%                 A_index=A_index-1;
%                 B_index=B_index-1;
%             end
%         else
%             B_index=temp(nearestIndex);
%             A_index=B_index-1;
%             if A_index<Vertex_index(sideIndex,1)
%                 A_index=A_index+1;
%                 B_index=B_index+1;
%             end
%         end
%   
%     end
%     
% end
nearestIndex=A_index;
[distance,flag]=point2line(p,coordinate(A_index,:),coordinate(B_index,:));

    
    