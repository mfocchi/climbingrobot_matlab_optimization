function [p, E,  path_length, initial_error, final_error] = eval_solution(x, Tf,  dt)
    global l m g OLD_FORMULATION   POLY_TYPE p0 pf
        %eval trajectory
        a_10 = x(1);
        a_11 = x(2);
        a_12 = x(3);
        a_13 = x(4);
        a_20 = x(5);
        a_21 = x(6);
        a_22 = x(7);
        a_23 = x(8);

    if (POLY_TYPE)  %quintic
        a_14 = x(9);
        a_15 = x(10);
        a_24 = x(11);
        a_25 = x(12);
    end
    
    t = linspace(0, Tf, 1/dt);
    
    
    if (OLD_FORMULATION)
           
        
       if (POLY_TYPE) %quintic
            theta = a_10 + a_11*t + a_12*t.^2 +  a_13*t.^3 + a_14*t.^4 + a_15*t.^5;
            thetad =  a_11 + 2*a_12*t + 3*a_13*t.^2 + 4*a_14*t.^3 + 5*a_15*t.^4 ;
            thetadd = 2*a_12 + 6*a_13*t + 12*a_14*t.^2  + 20*a_15*t.^3 ;

            phi = a_20 + a_21*t + a_22*t.^2 + a_23*t.^3 + a_24*t.^4 + a_25*t.^5;
            phid =  a_21 + 2*a_22*t  + 3*a_23*t.^2 + 4*a_24*t.^3 + 5*a_25*t.^4 ;
            phidd =   2*a_22 + 6*a_23*t + 12*a_24*t.^2  + 20*a_25*t.^3 ;
        else
            theta = a_10 + a_11*t + a_12*t.^2 +  a_13*t.^3;
            thetad =  a_11 + 2*a_12*t + 3*a_13*t.^2;
            thetadd = 2*a_12 + 6*a_13*t;

            phi = a_20 + a_21*t + a_22*t.^2 + a_23*t.^3;
            phid =  a_21 + 2*a_22*t  + 3*a_23*t.^2;
            phidd =   2*a_22 + 6*a_23*t;
        end

        p = [l*sin(theta).*cos(phi); l*sin(theta).*sin(phi); -l*cos(theta)]  ; 
        %disp(strcat('Initial velocity is [theta0, phi0]:',num2str(thetad(1)),"   ", num2str(phid(1))) );

    else

        z = a_10 + a_11*t + a_12*t.^2 +  a_13*t.^3;
        zd =  a_11 + 2*a_12*t + 3*a_13*t.^2;
        zdd = 2*a_12 + 6*a_13*t;

        phi = a_20 + a_21*t + a_22*t.^2 + a_23*t.^3;
        phid =  a_21 + 2*a_22*t  + 3*a_23*t.^2;
        phidd =   2*a_22 + 6*a_23*t;

        p = [sqrt(l^2 - z.^2).*cos(phi) ; sqrt(l^2 - z.^2).*sin(phi);   z];  
        
    end
    
    % velocity 
      pd = [cos(theta).*cos(phi).*thetad*l - sin(phi).*sin(theta).*phid*l;
          cos(theta).*sin(phi).*thetad *l + cos(phi).*sin(theta).*phid*l;
          sin(theta).*thetad*l];
    
      deltax = diff(p(1,:));  % diff(X);
      deltay = diff(p(2,:));   % diff(Y);
      deltaz = diff(p(3,:));    % diff(Z);    
      path_length = sum(sqrt(deltax.^2 + deltay.^2 + deltaz.^2));

    % check length is always l
%     a = vecnorm(p)
%     a -  ones(1,length(a))*l
    

    E = struct;
    

    % Calculating and ploting the total Energy from the new fit: theta, thetad and phid

    if (OLD_FORMULATION)
        E.Etot = ((m*l^2/2).*(thetad.^2 + sin(theta).^2 .*phid.^2)) - m*g*l.*cos(theta); 
 
    else
        E.Etot = m*(l^2).*(zd.^2)./(2*(l^2-z.^2)) + (m*(l^2-z.^2).*phid.^2)/2 + m*g.*z ;
        
    end
    
    
    % kinetic energy at the beginning
    E.Ekin0x = m/2*pd(1,1)'*pd(1,1);
    E.Ekin0y = m/2*pd(2,1)'*pd(2,1);
    E.Ekin0z = m/2*pd(3,1)'*pd(3,1);
    E.Ekin0 = m/2*pd(:,1)'*pd(:,1);
    E.U0 = -m*g*l*cos(theta(1));
       
    
    E.Ekinfx = m/2*pd(1,end)'*pd(1,end);
    E.Ekinfy = m/2*pd(2,end)'*pd(2,end);
    E.Ekinfz = m/2*pd(3,end)'*pd(3,end);
    E.Ekinf = m/2*pd(:,end)'*pd(:,end);
    E.Uf = -m*g*l*cos(theta(end));
      
    initial_error = norm(p(:,1) -p0);
    final_error = norm(p(:,end) -pf);
    
end