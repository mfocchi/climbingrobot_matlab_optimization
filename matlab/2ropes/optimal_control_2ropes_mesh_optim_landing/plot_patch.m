function  plot_patch(landing_patch_center,  params)

%make sure is column vector

landing_patch_center = landing_patch_center(:);
resolution = 0.1;
n_points = floor(params.patch_side / resolution);
dy = linspace(landing_patch_center(2)- params.patch_side/2, landing_patch_center(2)+ params.patch_side/2, n_points);
dz = linspace(landing_patch_center(3)- params.patch_side/2, landing_patch_center(3)+ params.patch_side/2, n_points);
for i=1:n_points
    
   
    for j=1:n_points
    
        point = [wallSurfaceEval(dz(j), dy(i), params),dy(i),  dz(j)];
        %initial
        plot3(point(1), point(2), point(3), 'Marker', '.', 'Color','b', 'MarkerSize',20) ; hold on;
    
    end
end
end
