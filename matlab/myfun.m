function yhat = myfun(beta,t)
    global l g  p0 pf  N Nbound_constr Nconstr 


    a_10 = beta(1);
    a_11 = beta(2);
    a_12 = beta(3);
    a_13 = beta(4);
    a_20 = beta(5);
    a_21 = beta(6);
    a_22 = beta(7);
    a_23 = beta(8);

    theta = a_10 + a_11*t + a_12*t.^2 +  a_13*t.^3;
    thetad =  a_11 + 2*a_12*t + 3*a_13*t.^2;
    thetadd = 2*a_12 + 6*a_13*t;

    phi = a_20 + a_21*t + a_22*t.^2 + a_23*t.^3;
    phid =  a_21 + 2*a_22*t  + 3*a_23*t.^2;
    phidd =   2*a_22 + 6*a_23*t;
    
    p_0 = [l*sin(theta(1)).*cos(phi(1)); l*sin(theta(1)).*sin(phi(1)); -l*cos(theta(1))];
    p_f = [l*sin(theta(N)).*cos(phi(N)); l*sin(theta(N)).*sin(phi(N)); -l*cos(theta(N))];  
    v_fy = cos(theta(N))*sin(phi(N))*l*thetad(N) - cos(phi(N))*sin(theta(N))*l*phid(N); %+ sin(theta(N))*sin(phi(N))*ldot
    yhat(1) = norm(p_0-p0);
    yhat(2) = norm(p_f-pf); 
    yhat(3) = abs(v_fy)
        
    erange = [2:N-1];
    
    Nenergy = Nconstr - Nbound_constr
    %energy derivative    
    yhat(Nbound_constr+1:Nbound_constr + Nenergy ) = l*( thetad(erange).*thetadd(erange)  + 2*sin(theta(erange) ).*cos(theta(erange)).*thetad(erange) .^2 +...
                    sin(theta(erange) ).^2 .* thetad(erange) .*thetadd(erange)  ) + g*sin(theta(erange) )
    yhat = yhat' ;              
    

end