

function draw_map_nav_ldm (landmarks,line_wid, configuration)
%-------------------------------------------------------
% University of Zaragoza
% Centro Politecnico Superior
% Robotics and Real Time Group
% Authors:  J. Neira, J. Tardos
% Date   :  7-2004
%-------------------------------------------------------
%-------------------------------------------------------
%global configuration;

%figure(configuration.ground); clf; axis equal; hold on;
% for i = 1: size (landmarks,1)
%     if(landmarks(i,5)<=80)%Type 10,20, ...,60 is manuveur point
%         plot(landmarks(i,[1 3]), landmarks(i,[2 4]),'-ro',...
%              'LineWidth',2,...
%              'MarkerSize', 2,...
%              'MarkerEdgeColor','c',...
%              'MarkerFaceColor','c');
%     %     rectangle('Position',[landmarks(i,1:4), ,'Curvature',0.2,...
%     %                'FaceColor',[0.5 0 0],'EdgeColor','r','LineWidth',5)
%     else
%         plot(landmarks(i,[1 3]), landmarks(i,[2 4]),'--yo',...
%              'LineWidth',2,...
%              'MarkerSize', 2,...
%              'MarkerEdgeColor','b',...
%              'MarkerFaceColor','y');
%         % rectangle('Position',landmarks(i,1:4),'Curvature',1,...
%         %            'FaceColor',[0.5 0 0.5],'EdgeColor','b','LineWidth',2);
%     end
% end
coder.extrinsic('num2str');
%   enumeration
%   XCrossing_Stop (11) % 十字路口停止�?
%   XCrossing_Start (12) % 十字路口目标车道进入�?
%   TCrossing_Stop (21)%T字路口停止线
%   TCrossing_Start (22)%T字路口目标车道进入线
    %   ZCrossing (30)%人行横道�?
%   Stop_1 (51)%边线1（沿车道横向边线），摆渡车停车点。停车点可能没有实际的标记，请按估计的停车点选取位置，比如添�?号楼门口的停车点
%   Stop_2 (52)%边线2（沿车道横向边线�?
%   LeftTurn_Stop (61)% 左转L形路口停止线
%   LeftTurn_Start (62)% 左转L形路口进入线
%   RightTurn_Stop (71)% 右转L形路口停止线
%   RightTurn_Start (72)% 右转L形路口进入线
    %   ParkSlotOnRoad_Stop (81)% 路边停车位边�?（沿车道横向边线�?
    %   ParkSlotOnRoad_Start (82)% 路边停车位边�?（沿车道横向边线�?
    %   ParkSlotOffRoad (90)%  路外停车位，只需测量沿道路行驶方向的�?��点和停止�?
    %   ParkSlotOffRoad_Start (91)%  路外停车位，只需测量沿道路行驶方向的�?���?
    %   ParkSlotOffRoad_Stop  (92)%  路外停车位，只需测量沿道路行驶方向的停止�?
%   RoadChange (100)% 车道宽度变化点，如两点间距离暗示车道宽度，则此时两点选取应保证，两点连线同测量处的车道垂直，或�?增加车道宽度的域�?
    %   SpeedLimit (110)%限�?标志，点类型
    %   GroundArrow (120)%地上箭头（参考EPM3的箭头位置定义），点类型
    %   TrafficLight (130)%交�?灯位置，点类�?
    %   UnClassified (180)%其他

for i = 1:size (landmarks,1)

    if(landmarks(i,5)==30 ||landmarks(i,5)==81 ||landmarks(i,5)==82 ||landmarks(i,5)==90 ||...
        landmarks(i,5)==91 ||landmarks(i,5)==92 ||landmarks(i,5)==110 ||landmarks(i,5)==120 ||...
        landmarks(i,5)==130 ||landmarks(i,5)==180)
        continue;
    end

    switch(landmarks(i,5))    %åœ°æ ‡ä¸­é¢œè�?²ç¬¦å·è¡¨ç¤ºå¯åœ¨ä¸‹æ�?¹ä¿®æ�?
    case(11) 
        color = 'ro';    %xcrossing_stop = tcrossing_stop + 1;
        lnm_label = 'XCe';
    case(12) 
        color = 'ro';    %xcrossing_start = tcrossing_stop + 1;
        lnm_label = 'XCb';
    case(21) 
        color = 'ro';    %tcrossing_stop = tcrossing_stop + 1;
        lnm_label = 'TCe';
    case(22)
        color = 'ro';    %tcrossing_start = tcrossing_start + 1;
        lnm_label = 'TCb';
    case(30)
        color = 'k';    %zcrossing = zcrossing + 1;
        lnm_label = 'ZC';
    case(51)   
        color = 'mo';    %Stop_1 = leftturn_stop + 1;
        lnm_label = 'STb';
    case(52)   
        color = 'mo';    %Stop_2 = leftturn_start + 1;
        lnm_label = 'STe';
    case(61)   
        color = 'ro';    %leftturn_stop = leftturn_stop + 1;
        lnm_label = 'LTe';
    case(62)   
        color = 'ro';    %rightturn_start = leftturn_start + 1;
        lnm_label = 'LTb';
    case(63)   
        color = 'bo';    %leftturn_stop = leftturn_stop + 1;
        lnm_label = 'LTe In';
    case(64)   
        color = 'bo';    %rightturn_start = leftturn_start + 1;
        lnm_label = 'LTb In';
    case(71)   
        color = 'ro';    %rightturn_stop = leftturn_stop + 1;
        lnm_label = 'RTe';
    case(72)   
        color = 'ro';    %leftturn_start = leftturn_start + 1;
        lnm_label = 'RTb';
    case(81)
        color = 'c';    %parkslotonroad_stop = parkslotonroad_stop + 1;
        lnm_label = 'PSLOn';
    case(82) 
        color = 'c';    %parkslotonroad_start = parkslotonroad_start + 1;
        lnm_label = 'PSLOn';
    case(90) 
        color = 'm';    %parkslotoffroad = parkslotoffroad + 1;
        lnm_label = 'PSLOff';
    case(91) 
        color = 'm';    %parkslotoffroad = parkslotoffroad + 1;
        lnm_label = 'PSLOff';
    case(92) 
        color = 'm';    %parkslotoffroad = parkslotoffroad + 1;
        lnm_label = 'PSLOff';
    case(100) 
        color = '*r';    %roadChange = speedLimit + 1;
        lnm_label = 'RWC';
    case(110) 
        color = '*r';    %speedLimit = speedLimit + 1;
        lnm_label = 'SLmt';
    case (120)   
        color = '*b';    %groundarrow = groundarrow + 1;
        lnm_label = 'Aw';
    case(130)
        color = '*g';    %trafficLight = trafficLight + 1;
        lnm_label = 'TLT';
    otherwise
        color = '*y';    %UnClassified (180)
        lnm_label = 'UnCl';
    end
    
    plot([landmarks(i,1),landmarks(i,3)],[landmarks(i,2),landmarks(i,4)],color,...
        'LineWidth',line_wid);

    x = (landmarks(i,1)+landmarks(i,3))/2;
    y = (landmarks(i,2)+landmarks(i,4))/2;
    lnm_label_1 = char([lnm_label,' ',num2str(i)]);
    text(x,y,lnm_label_1);    
end


