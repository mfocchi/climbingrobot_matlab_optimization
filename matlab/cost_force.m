function coste = cost(x)

    global dt num_params theta0 phi0 p0   pf w1  w2 w3 l
   
     
    x0 = [ theta0, phi0, 0, 0];
    u = x(1:num_params-1);
    Tf = x(num_params);  
    time = linspace(0, Tf, Tf/dt);
    
    p_vec(:,1) = p0;
    
    for i=1:length(time)
        [theta_vec(i), phi_vec(i), thetad_vec(i), phid_vec(i)] = integrate_dynamics(x0, u, time(i));
        p_vec(:,i) = [l*sin(theta_vec(i)).*cos(phi_vec(i)); l*sin(theta_vec(i)).*sin(phi_vec(i)); -l*cos(theta_vec(i))]; 
       
    end

    deltax = diff(p_vec(1,:));  % diff(X);
    deltay = diff(p_vec(2,:));   % diff(Y);
    deltaz = diff(p_vec(3,:));    % diff(Z);    
    path_length  = sum(sqrt(deltax.^2 + deltay.^2 + deltaz.^2));
    
   coste = w1*path_length + w2 * norm(p_vec(:,end) -pf)+ w3 * sum(x(num_params+1:end));
      
end