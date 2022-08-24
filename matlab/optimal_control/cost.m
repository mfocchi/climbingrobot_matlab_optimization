function coste = cost(x, p0,  pf)

    global m w1 w2 w3 w4 w5 N   num_params N_dyn dt_dyn

    thetad0 = x(1);
    phid0 = x(2);
    K = x(3);
    % Tf = x(4);
   
    
    [theta0, phi0, l_0] = computePolarVariables(p0);
    state0 = [theta0, phi0, l_0, thetad0, phid0, 0];
    [states, t] = integrate_dynamics(state0,dt_dyn, N_dyn, K);


    theta = states(1,:);
    phi = states(2,:);
    l = states(3,:);
    thetad = states(4,:);
    phid = states(5,:);
    ld = states(6,:);
    
    p = [l.*sin(theta).*cos(phi); l.*sin(theta).*sin(phi); -l.*cos(theta)];
    p_0 = p(:,1);
    p_f = p(:,end);
    l_f = l(end);
    

    % be careful there are only N values in this vector the path migh be
    % underestimated!
    deltax = diff(p(1,:));  % diff(X);
    deltay = diff(p(2,:));   % diff(Y);
    deltaz = diff(p(3,:));    % diff(Z);
    path_length = sum(sqrt(deltax.^2 + deltay.^2 + deltaz.^2));
    p_0 = p_0(:);
    p0 = p0(:);
    p_f= p_f(:);
    pf = pf(:);
    
    p0_cost = w1 * norm(p_0 - p0);
    pf_cost = w2 * norm(p_f -pf);
    lf_cost = w2*abs(norm(pf) - l_f);
    slack_energy= w3 * sum(x(num_params+1:num_params+N));
    slack_dyn = max(x(num_params+N+1:num_params+N+N_dyn));
    sigma_final_initial = w4 *sum (x(num_params+N+N_dyn + 1:end));
  
  
    Ekinfcost= w5 * (    (m*l(end)^2/2).*( thetad(end)^2 + sin(theta(end))^2 *phid(end)^2 )  + (m*ld(end)^2/2)   );


    %coste =  Tf  + Ekinfcost + slack_energy + sigma_final_initial ;
    coste =   Ekinfcost + 0.001* slack_energy + 0.001*slack_dyn + 0.001*sigma_final_initial ;

end