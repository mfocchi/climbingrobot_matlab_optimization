clear all;
close all;

loadFigOptions
load ('drilling.mat')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%margin -fx
imagesc(margin)
colormap turbo
a= colorbar('Location','eastoutside');
% caxis(colorbar_limits);
a.Label.Interpreter = 'latex';
a.Label.String = 'margin $-f_x$ [N]';
a.FontSize =40;

%//if you want you can Define a finer grid of points
% [z2,y2] = meshgrid(1:0.01:size(z,2), 1:0.01:size(y,2));
% %// Interpolate the data and show the output
% margin_interp = interp2(z, y, margin, z2, y2, 'linear');
% imagesc(margin_interp)

Npoints = size(z,2);
delta = Npoints /5;
set(gca, 'XTick', 0:delta:Npoints); 
set(gca, 'YTick', 0:delta:Npoints); 
set(gca, 'XTickLabel', linspace(min(y),max(y),6));
set(gca, 'YTickLabel', linspace(max(z),min(z),6));
grid on;
xlabel("Y" );
ylabel("Z" );

%save the plot
set(gcf, 'Paperunits' , 'centimeters')
set(gcf, 'PaperSize', [25 20]);
set(gcf, 'PaperPosition', [0 0 25 20]);
print(gcf, '-dpdf',strcat('drilling.pdf'),'-painters')




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%margin fz
load ('scaling_fz.mat')
figure
imagesc(margin_fz)
colormap turbo
a= colorbar('Location','eastoutside');
% caxis(colorbar_limits);
a.Label.Interpreter = 'latex';
a.Label.String = 'margin $f_z$ [N]';
a.FontSize =40;


Npoints = size(z,2);
delta = Npoints /5;
set(gca, 'XTick', 0:delta:Npoints); 
set(gca, 'YTick', 0:delta:Npoints); 
set(gca, 'XTickLabel', linspace(min(y),max(y),6));
set(gca, 'YTickLabel', linspace(max(z),min(z),6));
grid on;
xlabel("Y" );
ylabel("Z" );

%save the plot
set(gcf, 'Paperunits' , 'centimeters')
set(gcf, 'PaperSize', [25 20]);
set(gcf, 'PaperPosition', [0 0 25 20]);
print(gcf, '-dpdf',strcat('scaling_fz.pdf'),'-painters')



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%margin my
load ('scaling_my.mat')
figure
imagesc(margin_my)
colormap turbo
a= colorbar('Location','eastoutside');
% caxis(colorbar_limits);
a.Label.Interpreter = 'latex';
a.Label.String = 'margin $m_y$ [Nm]';
a.FontSize =40;

Npoints = size(z,2);
delta = Npoints /5;
set(gca, 'XTick', 0:delta:Npoints); 
set(gca, 'YTick', 0:delta:Npoints); 
set(gca, 'XTickLabel', linspace(min(y),max(y),6));
set(gca, 'YTickLabel', linspace(max(z),min(z),6));
grid on;
xlabel("Y" );
ylabel("Z" );

%save the plot
set(gcf, 'Paperunits' , 'centimeters')
set(gcf, 'PaperSize', [25 20]);
set(gcf, 'PaperPosition', [0 0 25 20]);
print(gcf, '-dpdf',strcat('scaling_my.pdf'),'-painters')


%//if you want you can Define a finer grid of points
% [z2,y2] = meshgrid(1:0.01:size(z,2), 1:0.01:size(y,2));
% %// Interpolate the data and show the output
% margin_interp = interp2(z, y, margin, z2, y2, 'linear');
% imagesc(margin_interp)


%with labels but I dont like it
%heatmap(y,z, margin)

%pcolor flip the matrix updown!
% Note that pcolor and imagesc will not display your data in the same way but the shading property is only available for faceted plots.
% pcolor(margin) 
% shading interp

%  you might want to normalize Z to range from 0 (or in the case of this data, to make the lowest non-zero value equal to zero) to 1 to improve the constrast:
% m = min(Z(Z~=0));
% M = max(Z(Z~=0));
% imshow((Z-m)/(M-m));
