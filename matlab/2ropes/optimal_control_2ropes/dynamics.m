function [dxdt] = dynamics(t, x, Fr_l, Fr_r,Fleg) % because we have time invariant system t wont be used

    global b g m
  
  
    %Retrieving states
    psi = x(1);
    l1 = x(2);
    l2 = x(3);
    psid = x(4);
    l1d = x(5);
    l2d = x(6);
    
    [px, py, pz]  = forwardKin(psi, l1, l2);  
    pz2b = pz*2*b;
    px2b = px*2*b;
    px_l1 = px/l1;
    n_pz_l1 =  -pz/l1;
    px_l1_sinpsi = px/l1/sin(psi);
    py2b = py*2*b;
    
    % mass equation and rope constraints 
    A_dyn = [l1*n_pz_l1,   px_l1 - (l1*sin(psi)*(py2b/(b^2*l1) - py2b^2/(2*b^2*l1^3)))/(2*px_l1_sinpsi),  (l2*py2b*sin(psi))/(2*b^2*l1*px_l1_sinpsi),
                      0,                                                                           l1/b,                                       -l2/b,
               l1*px_l1, (l1*cos(psi)*(py2b/(b^2*l1) - py2b^2/(2*b^2*l1^3)))/(2*px_l1_sinpsi) - n_pz_l1, -(l2*py2b*cos(psi))/(2*b^2*l1*px_l1_sinpsi)];
    
    
    b_dyn =  [2*l1d*n_pz_l1*psid - l1*psid^2*px_l1 - (sin(psi)*(4*l1^4*l1d^2 - 8*l1^3*l2*l1d*l2d + 4*l1^2*l2^2*l2d^2 - 6*l1^2*l1d^2*py2b - 2*l1^2*l2d^2*py2b + 8*l1*l2*l1d*l2d*py2b + 3*l1d^2*py2b^2))/(4*b^2*l1^3*px_l1_sinpsi) - (py2b^2*sin(psi)*(l1d*b^2 - l1d*l1^2 + 2*l2d*l1*l2 - l1d*l2^2)^2)/(16*b^4*l1^5*px_l1_sinpsi^3) + (psid*py2b*cos(psi)*(l1d*b^2 - l1d*l1^2 + 2*l2d*l1*l2 - l1d*l2^2))/(2*b^2*l1^2*px_l1_sinpsi) + (l1d*py2b*sin(psi)*(l1d*b^2 - l1d*l1^2 + 2*l2d*l1*l2 - l1d*l2^2))/(2*b^2*l1^3*px_l1_sinpsi),
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               (l1d^2 - l2d^2)/b,
               l1*n_pz_l1*psid^2 + 2*l1d*psid*px_l1 + (cos(psi)*(4*l1^4*l1d^2 - 8*l1^3*l2*l1d*l2d + 4*l1^2*l2^2*l2d^2 - 6*l1^2*l1d^2*py2b - 2*l1^2*l2d^2*py2b + 8*l1*l2*l1d*l2d*py2b + 3*l1d^2*py2b^2))/(4*b^2*l1^3*px_l1_sinpsi) + (py2b^2*cos(psi)*(l1d*b^2 - l1d*l1^2 + 2*l2d*l1*l2 - l1d*l2^2)^2)/(16*b^4*l1^5*px_l1_sinpsi^3) - (l1d*py2b*cos(psi)*(l1d*b^2 - l1d*l1^2 + 2*l2d*l1*l2 - l1d*l2^2))/(2*b^2*l1^3*px_l1_sinpsi) + (psid*py2b*sin(psi)*(l1d*b^2 - l1d*l1^2 + 2*l2d*l1*l2 - l1d*l2^2))/(2*b^2*l1^2*px_l1_sinpsi)];
 
        

    J =  computeJacobian([px; py; pz]);
    

    Ftot = [m*[0;0;-g] + J*[Fr_l;Fr_r] + evalImpulse(t,Fleg)]; 
    

    y = inv(A_dyn)*(inv(m)*Ftot - b_dyn);
    dxdt = [psid; l1d; l2d;  y];
    
end



