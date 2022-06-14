function [E_vec,  path_length, dist_to_target] = plot_curve_forceoptim(x, p0, pf, plot_energy, converged)
    global time num_params theta0 phi0   l m g
   
    x0 = [ theta0, phi0, 0, 0];
    u = x(1:num_params);
   
    p_vec(:,1) = p0;
    
        
    for i=1:length(time)
        [theta_vec(i), phi_vec(i), thetad_vec(i), phid_vec(i)] = integrate_dynamics(x0, u, time(i));
        p_vec(:,i) = [l*sin(theta_vec(i)).*cos(phi_vec(i)); l*sin(theta_vec(i)).*sin(phi_vec(i)); -l*cos(theta_vec(i))]; 
       
    end

        deltax = diff(p_vec(1,:));  % diff(X);
    deltay = diff(p_vec(2,:));   % diff(Y);
    deltaz = diff(p_vec(3,:));    % diff(Z);    
    path_length  = sum(sqrt(deltax.^2 + deltay.^2 + deltaz.^2));
    
    dist_to_target = norm(pf-p_vec(:,end))



    E_vec = ((m*l^2/2).*(thetad_vec.^2 + sin(theta_vec).^2 .*phid_vec.^2)) - m*g*l.*cos(theta_vec); 

     if (converged)

            plot3(p_vec(1,:), p_vec(2,:), p_vec(3,:),'r')   ;   
            hold on ;

     else
          plot3(p_vec(1,:), p_vec(2,:), p_vec(3,:) ,'k' ) ;   
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
   
    if (plot_energy)
        figure(2)
        plot(t,E(1,:))    
        title('Ploting the Energy')
        grid on
        xlabel('time');
        ylabel('Energy');
    end
    
end