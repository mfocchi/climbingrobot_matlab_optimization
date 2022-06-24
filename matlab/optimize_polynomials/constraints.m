function [ineq, eq] = constraints(x, l, time)

    global  g N  OLD_FORMULATION POLY_TYPE num_params
    % ineq are <= 0

    a_10 = x(1);
    a_11 = x(2);
    a_12 = x(3);
    a_13 = x(4);
    a_20 = x(5);
    a_21 = x(6);
    a_22 = x(7);
    a_23 = x(8);
        
    if (POLY_TYPE)  %quintic
        a_14 = x(9);
        a_15 = x(10);
        a_24 = x(11);
        a_25 = x(12);
    end
    
    for i=2:N-1 
        sigma(i) = x(num_params+i);
        
        if (OLD_FORMULATION)
            
            if (POLY_TYPE) %quintic
                theta = a_10 + a_11*time(i) + a_12*time(i).^2 +  a_13*time(i).^3 + a_14*time(i).^4 + a_15*time(i).^5;
                thetad =  a_11 + 2*a_12*time(i) + 3*a_13*time(i).^2 + 4*a_14*time(i).^3 + 5*a_15*time(i).^4 ;
                thetadd = 2*a_12 + 6*a_13*time(i) + 12*a_14*time(i).^2  + 20*a_15*time(i).^3 ;

                phi = a_20 + a_21*time(i) + a_22*time(i).^2 + a_23*time(i).^3 + a_24*time(i).^4 + a_25*time(i).^5;
                phid =  a_21 + 2*a_22*time(i)  + 3*a_23*time(i).^2 + 4*a_24*time(i).^3 + 5*a_25*time(i).^4 ;
                phidd =   2*a_22 + 6*a_23*time(i) + 12*a_24*time(i).^2  + 20*a_25*time(i).^3 ;
            else
                theta = a_10 + a_11*time(i) + a_12*time(i).^2 +  a_13*time(i).^3;
                thetad =  a_11 + 2*a_12*time(i) + 3*a_13*time(i).^2;
                thetadd = 2*a_12 + 6*a_13*time(i);

                phi = a_20 + a_21*time(i) + a_22*time(i).^2 + a_23*time(i).^3;
                phid =  a_21 + 2*a_22*time(i)  + 3*a_23*time(i).^2;
                phidd =   2*a_22 + 6*a_23*time(i);
            end
            
        
            ineq(i) = norm(l*( thetad*thetadd  + sin(theta)*cos(theta)*thetad*phid^2 +...
                        sin(theta)^2*phid*phidd) + g*sin(theta)*thetad) - sigma(i);

                
        else 
         
            z = a_10 + a_11*time(i) + a_12*time(i).^2 +  a_13*time(i).^3;
            zd =  a_11 + 2*a_12*time(i) + 3*a_13*time(i).^2;
            zdd = 2*a_12 + 6*a_13*time(i);
            phid =  a_21 + 2*a_22*time(i)  + 3*a_23*time(i).^2;
            phidd =   2*a_22 + 6*time(i);


            s_theta = sqrt(1 - z^2/l^2);
            c_theta = z/l;
            thetad = zd / sqrt(l^2 - z^2);
            thetadd = zd^2 / nthroot(l^2-z^2,3) + zdd / sqrt(l^2 - z^2);

%             ineq(i) = norm( l* (thetad * thetadd + 2+s_theta*c_theta * phid^2 + s_theta^2*phid*phidd) + g*s_theta) - sigma(i);
             ineq(i) = norm(l*( thetad*thetadd  + s_theta*c_theta*thetad*phid^2 +...
                        s_theta^2*phid*phidd) + g*s_theta*thetad) - sigma(i);
        end


    end

    eq = [];
end