

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

for i = 1:size (landmarks,1)
    switch(landmarks(i,5))    %地标中颜色符号表示可在下方修�?
    case (120)   
        color = '*b';    %groundarrow = groundarrow + 1;
    case(62)   
        color = 'g';    %leftturn_start = leftturn_start + 1;
    case(61)   
        color = 'm';    %leftturn_stop = leftturn_stop + 1;
    case(90) 
        color = 'm';    %parkslotoffroad = parkslotoffroad + 1;
    case(82) 
        color = 'c';    %parkslotonroad_start = parkslotonroad_start + 1;
    case(81)
        color = 'c';    %parkslotonroad_stop = parkslotonroad_stop + 1;
    case(110) 
        color = '*r';    %speedLimit = speedLimit + 1;
    case(22)
        color = 'y';    %tcrossing_start = tcrossing_start + 1;
    case(21) 
        color = 'y';    %tcrossing_stop = tcrossing_stop + 1;
    case(130)
        color = '*g';    %trafficLight = trafficLight + 1;
    case(30)
        color = 'k';    %zcrossing = zcrossing + 1;
    otherwise
        color = '*y';    %UnClassified (180)
    end
    
    plot([landmarks(i,1),landmarks(i,3)],[landmarks(i,2),landmarks(i,4)],color,...
        'LineWidth',line_wid);

    x = (landmarks(i,1)+landmarks(i,3))/2;
    y = (landmarks(i,2)+landmarks(i,4))/2;
    text(x,y,num2str(i));    

end


