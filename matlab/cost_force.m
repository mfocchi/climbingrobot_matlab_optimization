function coste = cost(x)

    global time num_params theta0 phi0 p0   pf w1 w2 w3 l
   
     
    x0 = [ theta0, phi0, 0, 0];
    u = x(1:num_params);
    p_vec(:,1) = p0;
    
    for i=1:length(time)
        [theta_vec(i), phi_vec(i), thetad_vec(i), phid_vec(i)] = integrate_dynamics(x0, u, time(i));
        p_vec(:,i) = [l*sin(theta_vec(i)).*cos(phi_vec(i)); l*sin(theta_vec(i)).*sin(phi_vec(i)); -l*cos(theta_vec(i))]; 
       
    end

    
   coste =  w2 * norm(p_vec(:,end) -pf)+ w3 * sum(x(7:end));
      
end