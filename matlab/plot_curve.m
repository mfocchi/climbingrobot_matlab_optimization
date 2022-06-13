function [E,  path_length] = plot_curve(x, Tf, p0, pf, dt ,plot_energy, converged)
    global l m g OLD_FORMULATION   POLY_TYPE
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
    
  %  for i=1:length(pd(1,:))  
   %     norm_pd(i) = sqrt(pd(1,i)^2 + pd(2,i)^2 + pd(3,i)^2);
        
  %  end
    
  %   path_length = sum(norm_pd*dt);
     
     
      deltax = diff(p(1,:));  % diff(X);
        deltay = diff(p(2,:));   % diff(Y);
        deltaz = diff(p(3,:));    % diff(Z);    
  path_length = sum(sqrt(deltax.^2 + deltay.^2 + deltaz.^2))

    % check length is always l
%     a = vecnorm(p)
%     a -  ones(1,length(a))*l
    
 if (converged)

        plot3(p(1,:), p(2,:), p(3,:),'r')   ;   
        hold on ;
       
 else
      plot3(p(1,:), p(2,:), p(3,:) ,'k' ) ;   
        hold on ;
        
        
 end
   plot3(p0(1), p0(2), p0(3), 'Marker', '.', 'Color','g', 'MarkerSize',60) ;
     plot3(pf(1), pf(2), pf(3), 'Marker', '.', 'Color','r', 'MarkerSize',60) ;
    grid on;
      view(3)
    xlim([-2 4])    
    ylim([-2 4])    
    zlim([-4 0])

    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    
    % Calculating and ploting the Energy from the new fit: theta, thetad and phid

    if (OLD_FORMULATION)
        E = ((m*l^2/2).*(thetad.^2 + sin(theta).^2 .*phid.^2)) - m*g*l.*cos(theta); 
 
    else
        E = m*(l^2).*(zd.^2)./(2*(l^2-z.^2)) + (m*(l^2-z.^2).*phid.^2)/2 + m*g.*z ;
        
    end
    
    
    if (plot_energy)
        figure(2)
        plot(t,E(1,:))    
        title('Ploting the Energy')
        grid on
        xlabel('time');
        ylabel('Energy');
    end
    
end