function  state = computeStateFromPosition(params, p,pd)


    l1 = norm(p - params.p_a1);
    l2 = norm(p - params.p_a2);


    
    if nargin >2 % we have also velocity
 
        J = computeJacobian(p, params);
        ld = pinv(J)*pd;
        l1d = ld(1);
        l2d = ld(2);   
    else
            pd = zeros(3,1);
            l1d = 0.0;
            l2d = 0.0;
    end
    
    state = [p; l1; l2; pd; l1d; l2d];   
    
end