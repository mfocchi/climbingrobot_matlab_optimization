
function cartesian_cone(x_vertex, y_vertex, z_vertex)


    %TODO
    coneHeight =  21;
   
    x = -5:0.25:5;    
    [xcone,ycone] = meshgrid(x);
    alpha = 30*pi/180;
    zcone  = -1/tan(alpha) * sqrt(xcone.^2+ycone.^2);
    surf(xcone,ycone,zcone,'FaceAlpha', 1.0, 'EdgeAlpha', 0.1 ,'EdgeColor','none');hold on;
    axis equal
    xlim([0, 5])    
    ylim([-5, 5])
    zlim([-6 0])
    
end