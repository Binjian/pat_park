function [distance,flag,nearestIndex]=calDistance(p,coordinate, Vertex_index,option)
% ����distance�Ǵ�ֱ���룬flag�Ƿ���1���ұߣ�0���غϣ�-1�����
% ����p�ǵ�ǰ����������
% ����offlineData���������ݵĽṹ�壬��coordinate��Vertex_index��
%        offlineData.coordinate�ǵ�ͼ����
%        offlineData.Vertex_index��ÿ�����ݶ��ڵö���
% option �Ǹ��ṹ�壨firstTime,oldIndex,clockWise��
%        ��� option.firstTime==1������Ҫ�ṩoption.oldIndex
%        ��� option.firstTime==0������Ҫ�ṩoption.oldIndex
%        option.clockWise=1��˳ʱ��;  =0����ʱ�룬


%%
flag = false;
% coordinate = offlineData.coordinate;
% Vertex_index = offlineData.Vertex_index;
d_side = zeros(1,4);
a = 0;
nearestIndex = 0;
for i=1:4                 %���㵱ǰ�������ߵ�ͼ���ĸ�����  sideIndex
    A=coordinate(Vertex_index(i,1),:);
    B=coordinate(Vertex_index(i,2),:);
   [d_side(i),flag]=point2line(p,A,B);
end
[a,sideIndex]=min(d_side);




%%  �״μ���
if option.firstTime==1
    d1 = zeros(1000,1);
%     d = zeros((Vertex_index(sideIndex,2) - Vertex_index(sideIndex,1)),1);
    for i=1:(Vertex_index(sideIndex,2) - Vertex_index(sideIndex,1))
        A=coordinate(Vertex_index(sideIndex,1)+i,:);
        d1(i)=squareDistance(p,A);
    end
    len = Vertex_index(sideIndex,2) - Vertex_index(sideIndex,1);
    d = d1(1:len);
    [a,nearestIndex]=min(d);  %������ڱ��ϵ�����ĵ�
    
    if nearestIndex==length(d);
        nearestIndex=nearestIndex-1;
    end
    if nearestIndex==1;
        nearestIndex=nearestIndex+1;
    end
    
    if d(nearestIndex+1)<d(nearestIndex-1)        
        if option.clockWise %˳ʱ��
            
            B_index=nearestIndex+Vertex_index(sideIndex,1);
            A_index=B_index+1;
            if A_index>Vertex_index(sideIndex,2)
                A_index=A_index-1;
                B_index=B_index-1;
            end
            
        else              %��ʱ��
            
            A_index=nearestIndex+Vertex_index(sideIndex,1);
            B_index=A_index+1;
            if B_index>Vertex_index(sideIndex,2)
                A_index=A_index-1;
                B_index=B_index-1;
            end
        end 
        
    else
        
        if option.clockWise %˳ʱ��
            A_index=nearestIndex+Vertex_index(sideIndex,1);
            B_index=A_index-1;
            if B_index<Vertex_index(sideIndex,1)
                A_index=A_index+1;
                B_index=B_index+1;
            end
        else          %��ʱ��
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
% %%  ���״μ���
% 
% %     oldIndex=option.oldIndex;
%     oldIndex = 0;
%     temp = zeros(21,1);
%     d = temp;
%     if option.clockWise %%% ���״� + ˳ʱ��
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
%     else   %% ���״� + ��ʱ��
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

    
    