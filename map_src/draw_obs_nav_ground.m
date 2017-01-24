%-------------------------------------------------------
% Patac
% SHD
% Software
% Authors:  Binjian Xin
% Date   :  2017-1
%-------------------------------------------------------

function draw_obs_nav_ground (object_list, sensor_data_raw,CIPO_id_lcs, CIPO_id_next_lcs, configuration)
%-------------------------------------------------------
%global configuration;

%draw fused observations
ob_id = object_list(:,1);
valid_fuse = find(ob_id);
x = object_list(valid_fuse,2);%observations.z(1:2:end);
y = object_list(valid_fuse,4);%observations.z(2:2:end);
plot(x, y, ['g' '+']);
%draw raw srr data;
ob_id = sensor_data_raw(:,1);
valid_srr = find(ob_id);
x = sensor_data_raw(valid_srr,2);%observations.z(1:2:end);
y = sensor_data_raw(valid_srr,3);%observations.z(2:2:end);
plot(x, y, ['b' 'x']);

if(CIPO_id_lcs(1,1)~=0)
    object_of_interest_id = CIPO_id_lcs(1,1);
    if(abs(object_of_interest_id)>0.5)
    	x = object_list(object_of_interest_id,2);%observations.z(1:2:end);
    	y = object_list(object_of_interest_id,4);%observations.z(2:2:end);
    	plot(x, y, 'rd','MarkerSize',10); 
    end
end

for p = 1:length(valid_fuse)
    if configuration.ellipses
        %draw object location ellipse
        %draw_ellipse (observations.z(2*p-1:2*p), observations.R(2*p-1:2*p, 2*p-1:2*p), 'g');
    % obs_pos = [object_list(valid_fuse(p),2),object_list(valid_fuse(p),4)];
    % obs_pos_cov = [object_list(valid_fuse(p),11), object_list(valid_fuse(p),13);...
    % 			   object_list(valid_fuse(p),19), object_list(valid_fuse(p),21)];
    % draw_ellipse (obs_pos, obs_pos_cov, 'g');

        %draw object velocity arrow and its covariance ellipse
	    initial_point=[object_list(valid_fuse(p),2),object_list(valid_fuse(p),4)];
	    end_point=initial_point+[object_list(valid_fuse(p),3),object_list(valid_fuse(p),5)];
	    generate_arrow(initial_point, end_point, 'c'); 
	    
        % obs_pos = (initial_point+end_point)/2;
        % obs_pos_cov = [object_list(valid_fuse(p),16), object_list(valid_fuse(p),18);...
        % 			   object_list(valid_fuse(p),24), object_list(valid_fuse(p),26)];
        % draw_ellipse (obs_pos, obs_pos_cov, 'g');

    end
%     if configuration.tags
%         ht = text(x(p)-0, y(p)+0.05, ['O' num2str(p)]);
%         set(ht, 'Color', 'g');
%     end  
end

