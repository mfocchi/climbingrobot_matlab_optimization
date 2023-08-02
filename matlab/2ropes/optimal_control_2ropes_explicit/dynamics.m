function [dxdt] = dynamics(t, x, Fr_l, Fr_r,Fleg, params) % because we have time invariant system t wont be used
     
  
    %Retrieving states
    px = x(1);
    py = x(2);
    pz = x(3);
    l1 = x(4);
    l2 = x(5);
    dpx = x(6);
    dpy = x(7);
    dpz = x(8);
    dl1 = x(9);
    dl2 = x(10);

    p = [px;py;pz];
    dp = [dpx;dpy;dpz];
    J =  computeJacobian([px; py; pz], params);
    
    % mass equation and rope constraints 
    A_dyn = [params.m*eye(3) ,    zeros(3,1)  , zeros(3,1),
         (p-params.p_a1)'  , -l1    ,   0,
         (p-params.p_a2)',   0    ,   -l2];

    
    Ftot = params.m*[0;0;-params.g] + J*[Fr_l;Fr_r]; 
    if norm(Fleg)>0
       Ftot= Ftot+ evalImpulse(t,Fleg,params); 
    end  
    
    b_dyn = [Ftot; -dp'*dp + dl1^2 ; -dp'*dp + dl2^2 ];     
    
    y = inv(A_dyn)*b_dyn;
    dxdt = [dpx; dpy; dpz; dl1; dl2; y];

    
end


