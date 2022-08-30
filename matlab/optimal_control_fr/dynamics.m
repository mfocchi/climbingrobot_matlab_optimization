function [dxdt] = dynamics(t, x, Fr) % because we have autonomous system t wont be used

    global m g l_uncompressed
    %Retrieving states
    theta = x(1); 
    phi = x(2);
    l = x(3);
    dtheta = x(4);
    dphi = x(5);
    dl = x(6);
    
     
    ddtheta = -2/l*(dtheta*dl) + cos(theta)*sin(theta)*(dphi^2)-(g/l)*sin(theta) ;
    ddphi = -2*(cos(theta)/sin(theta))*dphi*dtheta-(2/l)*dphi*dl ;
    ddl = l*(dtheta^2)+l*(sin(theta)^2)*dphi^2+g*cos(theta)+(1/m)*Fr;

    dxdt = [dtheta;  dphi;  dl; ddtheta; ddphi; ddl];
end