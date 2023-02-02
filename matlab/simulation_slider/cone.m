
function cone(x_vertex, y_vertex, z_vertex)


    Npoints = 40;
    coneHeight =  21;

  
    rCone = linspace(-coneHeight,0,40) ;
    thCone = linspace(pi/2,3/2*pi,Npoints) ;
  
    
    phiCone =3;
    
    %double cone
%     rCone = linspace(-20,20,40) ;
%     thCone = linspace(0,2*pi,Npoints) ;
%     
    
    [RCone,TCone] = meshgrid(rCone,thCone) ;
    XCone = RCone.*cos(TCone)*sin(phiCone)+x_vertex ;
    YCone = RCone.*sin(TCone)*sin(phiCone)+y_vertex ;
    ZCone = RCone  +z_vertex;
  
    radius = sqrt( (rCone(1) *cos(thCone(1))*sin(phiCone))^2 +  (rCone(1) *sin(thCone(1))*sin(phiCone))^2);
    

    %%%%%
    surf(XCone,YCone,ZCone,'FaceAlpha', 1.0, 'EdgeAlpha', 0.1 ,'EdgeColor','none');hold on;
    axis equal
    
    
end