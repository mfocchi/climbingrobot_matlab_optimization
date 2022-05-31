function coste = cost(x)

    global N l p0 pf w1 w2 w3 time OLD_FORMULATION POLY_TYPE

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

    coste = w1 * norm(p_0 - p0) + w2 * norm(p_f -pf)+ w3 * sum(x(9:end));
   
   
end