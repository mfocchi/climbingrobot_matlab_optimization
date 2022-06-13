function coste = cost(x)

    global Tf l theta0 phi0 p0   pf w1 w2 w3 dt
   
     
         
    mea1 = x(1);
    sig1 = x(2);
    fn0 = x(3);
    mea2 = x(4);
    sig2 = x(5);
    ft0 = x(6);
    
    theta_vec(1) = theta0;
    phi_vec(1)= phi0;
    dtheta_vec(1) = 0;
    dphi_vec(1) = 0;
    ddtheta_vec(1) = 0;
    ddphi_vec(1) = 0;
    time = 0;
    p_vec(:,1) = p0;
    
    
    for i=2:length(Tf)
        

        fun = fn0.*(1./(sqrt(2.*pi.*sig1.^2)))*exp(-0.5.*((time-mea1)./sig1).^2);
        fut = ft0.*(1./(sqrt(2.*pi.*sig2.^2)))*exp(-0.5.*((time-mea2)./sig2).^2);

        ddtheta_vec(i) = cos(theta_vec(i)).*sin(theta_vec(i)).*(dphi_vec(i).^2)-(g./l).*sin(theta_vec(i))+fun./(m.*l);
        ddphi_vec(i) = -2.*(cos(theta_vec(i))./sin(theta_vec(i))).*dphi_vec(i).*dtheta_vec(i) + fut./(m.*l.*sin(theta_vec(i)));

        dtheta_vec(i) = ddtheta_vec(i-1).*dt ;
        dphi_vec(i) =  dphi_vec(i-1) + ddphi_vec(i).*dt ;

        theta_vec(i) = theta_vec(i-1)  + dtheta_vec(i-1).*dt + 0.5*ddtheta_vec(i)*dt.^2;
        phi_vec(i) = phi_vec(i-1)  + dphi_vec(i-1).*dt + 0.5*ddphi_vec(i)*dt.^2;


        p_vec(:,i) = [l*sin(theta_vec(i)).*cos(phi_vec(i)); l*sin(theta_vec(i)).*sin(phi_vec(i)); -l*cos(theta_vec(i))]; 
        time = time + dt;
    end

    
   coste = w1 * norm(p_vec(1,:) - p0) + w2 * norm(p_vec(:,end) -pf)+ w3 * sum(x(7:end));
   
   
end