function [px, py, pz] = forwardKin(psi, l1, l2)
global b 
    px =     l1*sin(psi)*(1 - (b^2 + l1^2 - l2^2)^2/(4*b^2*l1^2))^(1/2);
    py =         (b^2 + l1^2 - l2^2)/(2*b);
    pz =     -l1*cos(psi)*(1 - (b^2 + l1^2 - l2^2)^2/(4*b^2*l1^2))^(1/2);                                 

end