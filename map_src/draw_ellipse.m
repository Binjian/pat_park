%-------------------------------------------------------
% Patac
% SHD
% Software
%-------------------------------------------------------
% \author:  Binjian Xin
% \Date   :  2017-1
%
% \brief calculates the inverse of one or more transformations
%-------------------------------------------------------

 function h = draw_ellipse(pos, cov, color)
global chi2;
coder.extrinsic('full');
persistent CIRCLE

if isempty(CIRCLE) 
    tita = linspace(0, 2*pi,40);
    CIRCLE = [cos(tita); sin(tita)];
end

%[V,D]=eig(full(cov(1:2,1:2)));
[V,D]=eig(cov(1:2,1:2));
ejes=sqrt(chi2(2)*diag(D));
P = (V*diag(ejes))*CIRCLE;
hp = line(P(1,:)+pos(1), P(2,:)+pos(2));
set(hp,'Color', color);
%set(hp, 'LineWidth', 1.5);
