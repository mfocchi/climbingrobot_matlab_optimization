function [ineq, eq] = constraints(x, l)

global  g N    num_params

% ineq are <= 0
Tf = x(1);
time =  linspace(0, Tf, N) ;
a_10 = x(2);
a_11 = x(3);
a_12 = x(4);
a_13 = x(5);
a_20 = x(6);
a_21 = x(7);
a_22 = x(8);
a_23 = x(9);


for i=2:N-1
    sigma(i) = x(num_params+i);
    
    
    theta = a_10 + a_11*time(i) + a_12*time(i).^2 +  a_13*time(i).^3;
    thetad =  a_11 + 2*a_12*time(i) + 3*a_13*time(i).^2;
    thetadd = 2*a_12 + 6*a_13*time(i);
    
    phi = a_20 + a_21*time(i) + a_22*time(i).^2 + a_23*time(i).^3;
    phid =  a_21 + 2*a_22*time(i)  + 3*a_23*time(i).^2;
    phidd =   2*a_22 + 6*a_23*time(i);
    
    
    %             ineq(i) = norm(l*( thetad*thetadd  + 2*sin(theta)*cos(theta)*thetad^2 +...
    %                         sin(theta)^2.*thetad*thetadd) + g*sin(theta)) - sigma(i);
    ineq(i) = norm(l*( thetad*thetadd  + sin(theta)*cos(theta)*thetad*phid^2 +...
        sin(theta)^2*phid*phidd) + g*sin(theta)*thetad) - sigma(i);
    
    
    
    
    
end

eq = [];
end