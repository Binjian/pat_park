%plot(middleLine.coordinate(:,1),middleLine.coordinate(:,2));
coordinate = middleLine.coordinate;



%���еر�
Landmark_All = [GroundArrow; LeftTurn_Start; LeftTurn_Stop; ParkSlotOffRoad; ParkSlotOnRoad_Start;
    ParkSlotOnRoad_Stop; SpeedLimit; TCrossing_Start;  TCrossing_Stop; TrafficLight; ZCrossing];

[Num_Landmark,i]=size(Landmark_All);
[Num_coordinate,i]=size(coordinate);

%middleline���
for i = 1:Num_coordinate
    coordinate(i,3) = i;
end

%��ر괦��
for i = 1:Num_Landmark
    if (Landmark_All(i,3) == 0) & (Landmark_All(i,4) == 0)
        Landmark_All(i,3) = Landmark_All(i,1);
        Landmark_All(i,4) = Landmark_All(i,2);
    end
end
        
        
%���ɾ���
for i = 1:Num_Landmark
    for j=1:Num_coordinate
        distence(i,j) = sqrt(coordinate(j,1)-0.5*(Landmark_All(i,1)+Landmark_All(i,3))) + sqrt(coordinate(j,1)-0.5*(Landmark_All(i,2)+Landmark_All(i,4)));
    end
end

%�ر��� 
for i = 1:Num_Landmark
    [cor_distence,cor_index] = min(distence(i,:));
    Landmark_All(i,6) = cor_index;
   
end

%{
plot(Landmark_All(:,1),Landmark_All(:,2),'.'); 
hold on
plot(Landmark_All(:,3),Landmark_All(:,4),'.r'); 
%}


%�ر�������� 
groundarrow = 0;             %�� 120 ��ɫ *b     
leftturn_start = 0;          %�� 62  ��ɫ g         
leftturn_stop = 0;           %�� 61  �Ϻ� m
parkslotoffroad = 0;         %�� 90  �Ϻ� m 
parkslotonroad_start = 0;    %�� 82  ���� c
parkslotonroad_stop = 0;      %�� 81  ���� c
speedLimit = 0;              %�� 110 ��ɫ *r
tcrossing_start = 0;         %�� 22  ��ɫ y
tcrossing_stop = 0;          %�� 21  ��ɫ y
trafficLight = 0;            %�� 130 ��ɫ *g
zcrossing = 0;               %�� 30  ��ɫ k

for i = 1:Num_Landmark
    switch(Landmark_All(i,5))    %�ر�����ɫ���ű�ʾ�����·��޸�
    case (120)   
        color = '*b';    groundarrow = groundarrow + 1;
    case(62)   
        color = 'g';    leftturn_start = leftturn_start + 1;
    case(61)   
        color = 'm';    leftturn_stop = leftturn_stop + 1;
    case(90) 
        color = 'm';    parkslotoffroad = parkslotoffroad + 1;
    case(82) 
        color = 'c';    parkslotonroad_start = parkslotonroad_start + 1;
    case(81)
        color = 'c';    parkslotonroad_stop = parkslotonroad_stop + 1;
    case(110) 
        color = '*r';    speedLimit = speedLimit + 1;
    case(22)
        color = 'y';    tcrossing_start = tcrossing_start + 1;
    case(21) 
        color = 'y';    tcrossing_stop = tcrossing_stop + 1;
    case(130)
        color = '*g';    trafficLight = trafficLight + 1;
    case(30)
        color = 'k';    zcrossing = zcrossing + 1;
    end
    
    plot([Landmark_All(i,1),Landmark_All(i,3)],[Landmark_All(i,2),Landmark_All(i,4)],color);
    hold on
end

   









     


