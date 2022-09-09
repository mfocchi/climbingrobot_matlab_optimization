
function cone(x_vertex, y_vertex, z_vertex)


    Npoints = 40;
    coneHeight =  21;

    coneBaseZ = z_vertex - coneHeight;
    

    rCone = linspace(coneBaseZ,z_vertex,40) ;
    thCone = linspace(pi/2,3/2*pi,Npoints) ;
  
    
    phiCone =3;
    
    %double cone
%     rCone = linspace(-20,20,40) ;
%     thCone = linspace(0,2*pi,Npoints) ;
%     
    
    [RCone,TCone] = meshgrid(rCone,thCone) ;
    XCone = RCone.*cos(TCone)*sin(phiCone)+x_vertex ;
    YCone = RCone.*sin(TCone)*sin(phiCone)+y_vertex ;
    ZCone = RCone ;
  
    radius = sqrt( (rCone(1) *cos(thCone(1))*sin(phiCone))^2 +  (rCone(1) *sin(thCone(1))*sin(phiCone))^2);
    

    %%%%%
    surf(XCone,YCone,ZCone);
    axis equal
    
    
end