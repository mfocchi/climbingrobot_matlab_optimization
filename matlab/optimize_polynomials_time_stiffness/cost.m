function coste = cost(x, p0,  pf)

    global m w1 w2 w3 w4 w5 N   num_params


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
    theta = a_10 + a_11*time + a_12*time.^2 +  a_13*time.^3;
    phi = a_20 + a_21*time + a_22*time.^2 + a_23*time.^3;
    l = a_30 + a_31*time + a_32*time.^2 + a_33*time.^3;
    thetad =  a_11 + 2*a_12*time + 3*a_13*time.^2;
    phid =  a_21 + 2*a_22*time  + 3*a_23*time.^2;
    ld =  a_31 + 2*a_32*time  + 3*a_33*time.^2; 
    
    p = [l.*sin(theta).*cos(phi); l.*sin(theta).*sin(phi); -l.*cos(theta)];
    p_0 = p(:,1);
    p_f = p(:,end);
    l_f = l(end);
    
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
    lf_cost = w2*abs(norm(pf) - l_f);
    slack_cost= w3 * sum(x(num_params+1:num_params+N));
    sigma_final_initial = w4 *sum (x(num_params+N+1:end));
   
    Ekin0cost= w5 * (    (m*l(1)^2/2).*( thetad(1)^2 + sin(theta(1))^2 *phid(1)^2 )  + (m*ld(1)^2/2)   );
    
    fprintf('cost comparison p0: %5.2f  pf: %5.2f  lf: %5.2f  slack_cost: %5.2f  Ekin0_cost: %5.2f \n' , norm(p_0 - p0), norm(p_f - pf),  abs(norm(pf) - l_f), sum(x(num_params+1:num_params+N)), (m*l(1)^2/2).*( thetad(1)^2 + sin(theta(1))^2 *phid(1)^2 ));
  
    coste =  Ekin0cost + slack_cost +sigma_final_initial ;
%      coste =    sigma_final_initial  + slack_cost ;
   

end