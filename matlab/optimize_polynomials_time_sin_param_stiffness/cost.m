function coste = cost(x, p0,  pf)

    global w1 w2 w3 w4 N   num_params


    Tf = x(1);
    time =  linspace(0, Tf, N) ;
    a_10 = x(2);
    a_11 = x(3);
    a_12 = x(4);
    a_13 = x(5);
    a_20 = x(6);
    a_21 = x(7);
    a_22 = x(8);
    a_23 = x(9);
    a_30 = x(10);
    a_31 = x(11);
    a_32 = x(12);
    a_33 = x(13);
    K = x(14);
   

    % parametrizzation with sin(theta) sin(phi)
    s_theta = a_10 + a_11*time + a_12*time.^2 +  a_13*time.^3;
    c_theta = sqrt(1 -  s_theta.^2);
    s_phi = a_20 + a_21*time + a_22*time.^2 + a_23*time.^3;
    c_phi = sqrt(1 -  s_phi.^2);
    l = a_30 + a_31*time + a_32*time.^2 + a_33*time.^3;
    
    
     
    p = [l.*s_theta.*c_phi; l.*s_theta.*s_phi; -l.*c_theta];
    p_0 = p(:,1);
    p_f = p(:,end);
    
    %count row wise how many elemnts are lower than zero
    negative_el = sum(p<0, 2);
    x_inside_wall = negative_el(1);

    % be careful there are only N values in this vector the path migh be
    % underestimated!
    deltax = diff(p(1,:));  % diff(X);
    deltay = diff(p(2,:));   % diff(Y);
    deltaz = diff(p(3,:));    % diff(Z);
    path_length = sum(sqrt(deltax.^2 + deltay.^2 + deltaz.^2));

    p0_cost = w1 * norm(p_0 - p0);
    pf_cost = w2 * norm(p_f -pf);
%     slack_cost= w3 * sum(x(num_params+1:end));
    slack_cost= w3 * sum(x(num_params+1:N));
    
    sigma_final_initial = w4 *sum (x(num_params+N+1:end));
    
     wall_cost =  1000*x_inside_wall;

%     theta0 = a_10;
%     thetad0 = a_11;
%     phid0 = a_21;
    %Ekin0cost= w4 * (m*l^2/2).*(thetad0^2 + sin(theta0)^2 *phid0^2);
    
%     coste =   slack_cost +  sigma_final_initial;

%       coste =  p0_cost  + pf_cost   + slack_cost ;
      coste =  p0_cost  + pf_cost   + slack_cost + sigma_final_initial;
   

end