function coste = cost(x)

    global   l p0 pf w1 w2 w3 time OLD_FORMULATION POLY_TYPE num_params

    a_10 = x(1);
    a_11 = x(2);
    a_12 = x(3);
    a_13 = x(4);
    a_20 = x(5);
    a_21 = x(6);
    a_22 = x(7);
    a_23 = x(8);
    
    if (POLY_TYPE) %quintic
        a_14 = x(9);
        a_15 = x(10);
        a_24 = x(11);
        a_25 = x(12);
    end

    
    if (OLD_FORMULATION)
        if (POLY_TYPE) %quintic
            theta0 = a_10 + a_11*time(1) + a_12*time(1).^2 +  a_13*time(1).^3 + a_14*time(1).^4 + a_15*time(1).^5;
            phi0 = a_20 + a_21*time(1) + a_22*time(1).^2 + a_23*time(1).^3 + a_24*time(1).^4 + a_25*time(1).^5;
            thetaf = a_10 + a_11*time(end) + a_12*time(end).^2 +  a_13*time(end).^3 + a_14*time(end).^4 + a_15*time(end).^5;
            phif = a_20 + a_21*time(end) + a_22*time(end).^2 + a_23*time(end).^3 + a_24*time(end).^4 + a_25*time(end).^5;
      
        else
            theta0 = a_10 + a_11*time(1) + a_12*time(1).^2 +  a_13*time(1).^3;
            phi0 = a_20 + a_21*time(1) + a_22*time(1).^2 + a_23*time(1).^3;
            thetaf = a_10 + a_11*time(end) + a_12*time(end).^2 +  a_13*time(end).^3;
            phif = a_20 + a_21*time(end) + a_22*time(end).^2 + a_23*time(end).^3;
        end
        
        p_0 = [l*sin(theta0).*cos(phi0); l*sin(theta0).*sin(phi0); -l*cos(theta0)];
        p_f = [l*sin(thetaf).*cos(phif); l*sin(thetaf).*sin(phif); -l*cos(thetaf)]; 
    else 
        
        phi0 = a_20 + a_21*time(1) + a_22*time(1).^2 + a_23*time(1).^3 ;   
        z0 = a_10 + a_11*time(1) + a_12*time(1).^2 +  a_13*time(1).^3;
        y0 = sqrt(l^2 -z0^2) *  sin(phi0);
        x0 =  sqrt(l^2 -z0^2) *  cos(phi0);
        p_0 = [x0; y0; z0];


        phif = a_20 + a_21*time(end) + a_22*time(end).^2 + a_23*time(end).^3;
        zf = a_10 + a_11*time(end) + a_12*time(end).^2 +  a_13*time(end).^3;
        yf = sqrt(l^2 -z0^2) *  sin(phif);
        xf =  sqrt(l^2 -z0^2) *  cos(phif);
        p_f = [xf; yf; zf];
    
 
    end
    
    
            
    if (POLY_TYPE) %quintic
        theta = a_10 + a_11*time + a_12*time.^2 +  a_13*time.^3 + a_14*time.^4 + a_15*time.^5;
        phi = a_20 + a_21*time + a_22*time.^2 + a_23*time.^3 + a_24*time.^4 + a_25*time.^5;
       
    else
        theta = a_10 + a_11*time + a_12*time.^2 +  a_13*time.^3;
        phi = a_20 + a_21*time + a_22*time.^2 + a_23*time.^3;
        
    end
    p = [l*sin(theta).*cos(phi); l*sin(theta).*sin(phi); -l*cos(theta)]; 
    
    %count row wise how many elemnts are lower than zero
    negative_el = sum(p<0, 2); 
    x_inside_wall = negative_el(1);
    

       deltax = diff(p(1,:));  % diff(X);
      deltay = diff(p(2,:));   % diff(Y);
      deltaz = diff(p(3,:));    % diff(Z);    
      path_length = sum(sqrt(deltax.^2 + deltay.^2 + deltaz.^2));
      
      p0_cost = w1 * norm(p_0 - p0);
      pf_cost = w2 * norm(p_f -pf);
      slack_cost= w3 * sum(x(num_params+1:end));
      wall_cost =  1000*x_inside_wall;
      
    coste = wall_cost  + p0_cost  + pf_cost + slack_cost;
    
   
end