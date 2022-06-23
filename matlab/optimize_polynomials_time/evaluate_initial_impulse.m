function [Fun , Fut] = evaluate_initial_impulse(x, t, l)
    global  m     
    %eval trajectory
        a_10 = x(1);
        a_11 = x(2);
        a_12 = x(3);
        a_13 = x(4);
        a_20 = x(5);
        a_21 = x(6);
        a_22 = x(7);
        a_23 = x(8);

   
%        
%     if (OLD_FORMULATION)          
%         
%        if (POLY_TYPE) %quintic
%             theta0 = a_10 + a_11*t + a_12*t.^2 +  a_13*t.^3 + a_14*t.^4 + a_15*t.^5;
%             thetad0 =  a_11 + 2*a_12*t + 3*a_13*t.^2 + 4*a_14*t.^3 + 5*a_15*t.^4 ;           
%             phid =  a_21 + 2*a_22*t  + 3*a_23*t.^2 + 4*a_24*t.^3 + 5*a_25*t.^4 ;
%           
%        else
%             theta0 = a_10 + a_11*t + a_12*t.^2 +  a_13*t.^3;
%             thetad0 =  a_11 + 2*a_12*t + 3*a_13*t.^2;
%             phid0 =  a_21 + 2*a_22*t  + 3*a_23*t.^2;
% 
%         end
%    
%     else
%      
%         zd0 =  a_11 + 2*a_12*t + 3*a_13*t.^2;
%         phid0 =  a_21 + 2*a_22*t  + 3*a_23*t.^2;
% 
%     end
    theta0 = a_10;
    thetad0 = a_11;
    phid0 = a_21;
    %evaluate inpulse ( the integral of the gaussian is 1) 
    Fun = m*l*thetad0;
    Fut = m*l*sin(theta0)*phid0;
      
end