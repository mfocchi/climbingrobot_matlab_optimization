function [Fun , Fut] = evaluate_initial_impulse_mj(x)
    global  m    
   
    %eval trajectory
    Tf = x(1);
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
       
        
  % parametrizzation with sin theta sing phi
    
    theta0 = a_10;
    thetad0 = a_11;
    phid0 = a_21;
    l0 = a_30;

    %evaluate inpulse ( the integral of the gaussian is 1) 
    Fun = m*l0*thetad0;
    Fut = m*l0*sin(theta0)*phid0;

      
end