function [Fun , Fut] = evaluate_initial_impulse(x, t, l)
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
        
  % parametrizzation with sin theta sing phi
    arg1_0 = a_10 ; 
    arg1d_0 = a_11 ;
    arg2_0 = a_20; 
    arg2d_0 =  a_21 ;
    s_theta0 = arg1_0;
    
    theta0 = asin(s_theta0);
    phid0 = 1/sqrt(1-arg2_0)*arg2d_0;
        
    thetad0 = a_11;
    phid0 = a_21;
    %evaluate inpulse ( the integral of the gaussian is 1) 
    Fun = m*l*thetad0;
    Fut = m*l*sin(theta0)*phid0;
      
end