
function [theta, phi, thetad, phid, thetadd, phidd] =  integrate_dynamics(x0, u, final_time)
    global dt l g m

    theta  = x0(1);
    phi = x0(2);
    thetad = x0(3);
    phid = x0(4);  
    
    mean  = u(1);
    sig = mean/3;
    fn0 = u(2);
    ft0 = u(3);  
       
    
    time = 0;
    
    while time<=final_time
        fun  = fn0.*(1./(sqrt(2.*pi.*sig.^2)))*exp(-0.5.*((time-mean)./sig).^2);
        fut  = ft0.*(1./(sqrt(2.*pi.*sig.^2)))*exp(-0.5.*((time-mean)./sig).^2);
        
        thetadd = cos(theta).*sin(theta).*(phid.^2)-(g./l).*sin(theta)+fun ./(m.*l);
        phidd = -2.*(cos(theta)./sin(theta)).*phid.*thetad + fut./(m.*l.*sin(theta));
     
        
        thetad = thetad + thetadd.*dt ;
        phid = phid + phidd.*dt ;
    
        theta = theta   + thetad.*dt + 0.5*thetadd*dt.^2;
        phi = phi   + phid.*dt + 0.5*phidd*dt.^2;
        time = time + dt;
    end
    
end