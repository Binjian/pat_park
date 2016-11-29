function draw_tables (compatibility, GT, H, configuration)
%-------------------------------------------------------
% University of Zaragoza
% Centro Politecnico Superior
% Robotics and Real Time Group
% Authors:  J. Neira, J. Tardos
% Date   :  7-2004
%-------------------------------------------------------
%-------------------------------------------------------
%global configuration;
coder.extrinsic('imagesc');
if configuration.step_by_step

%ncol=1000;
ncol=256 ;
mapa=hsv2rgb([linspace(2/3, 0, ncol)' 0.9*ones(ncol,1) ones(ncol,1)]) ;
mapa(1,:)=[1 1 1] ;
mapa = flipdim(mapa,1);
%colormap(mapa) ;

colormap winter
m = colormap;
%m(1,:)=[1 1 1] ;

figure(configuration.tables); clf; 
subplot(2,2,1);
colormap(m); 
table = compatibility.ic(:, compatibility.candidates.features);
%table = compatibility.d2(:, compatibility.candidates.features);
%table = 1./table;
%table = table .* compatibility.ic(:, compatibility.candidates.features);
imagesc(table);
%colorbar;
title('INDIVIDUAL COMPATIBILITY');

subplot(2,2,2);
colormap(m);
GT
table = make_table(compatibility, GT);
imagesc(table);
%colorbar;
title('GROUND TRUTH');

subplot(2,2,3);
colormap(m);
table = make_table(compatibility, H);
imagesc(table);
%colorbar;
% title(configuration.name);

pause
end

