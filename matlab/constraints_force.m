function [ineq, eq] = constraints_force(x)

    global  g theta0 phi0 mu l Fun_max Tf


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
       
    for i=2:length(Tf)        

        fun(i) = fn0.*(1./(sqrt(2.*pi.*sig1.^2)))*exp(-0.5.*((time-mea1)./sig1).^2);
        fut(i) = ft0.*(1./(sqrt(2.*pi.*sig2.^2)))*exp(-0.5.*((time-mea2)./sig2).^2);

        ddtheta_vec(i) = cos(theta_vec(i)).*sin(theta_vec(i)).*(dphi_vec(i).^2)-(g./l).*sin(theta_vec(i))+fun./(m.*l);
        ddphi_vec(i) = -2.*(cos(theta_vec(i))./sin(theta_vec(i))).*dphi_vec(i).*dtheta_vec(i) + fut./(m.*l.*sin(theta_vec(i)));

        dtheta_vec(i) = ddtheta_vec(i-1).*dt ;
        dphi_vec(i) =  dphi_vec(i-1) + ddphi_vec(i).*dt ;

        theta_vec(i) = theta_vec(i-1)  + dtheta_vec(i-1).*dt + 0.5*ddtheta_vec(i)*dt.^2;
        phi_vec(i) = phi_vec(i-1)  + dphi_vec(i-1).*dt + 0.5*ddphi_vec(i)*dt.^2;


        p_vec(:,i) = [l*sin(theta_vec(i)).*cos(phi_vec(i)); l*sin(theta_vec(i)).*sin(phi_vec(i)); -l*cos(theta_vec(i))]; 
        time = time + dt;
    end

    
    
     
    % force constraints: friction ft0 <= mu *fn0;

    ineq(1) = abs(ft0) - mu *fn0;
    ineq(2) = -fn0;
    ineq(3) = fn0 -Fun_max;
    
    
%      sigma = x(num_params: num_params+N);
%      ineq(4:4+N) = norm(l*( dtheta_vec.*ddtheta_vec  + sin(theta_vec).*cos(theta_vec).*dtheta_vec.*dphi_vec^2 +...
%                         sin(theta_vec).^2*dphi_vec.* ddphi_vec) + g*sin(theta_vec).*dtheta_vec) - sigma;


     
    eq = [];
end