function [ineq, eq] = constraints_force(x)

    global  g Tf theta0 phi0 mu l Fun_max N num_params impulse_mean

    fn0=x(1);
    ft0 = x(2);
     
    x0 = [ theta0, phi0, 0, 0];
    
    u = x(1:num_params);  
    sigma = x(num_params+1: num_params+N);
    
     
    % force constraints: friction ft0 <= mu *fn0;
    ineq(1) = abs(ft0) - mu *fn0;
    ineq(2) = -fn0; %fn >= 0
    ineq(3) = fn0 - Fun_max;%fn <= fnmax      
    
    time_instants_energy = linspace(impulse_mean*3, Tf, N); % energy will have to conserve only after the application of the force
    
    for i=1:N
        [theta , phi , thetad , phid , thetadd, phidd] = integrate_dynamics(x0, u, time_instants_energy(i));     
        norm_energy_dot = norm(l*( thetad.*thetadd  + sin(theta).*cos(theta).*thetad.*phid^2 +...
                        sin(theta ).^2*phid.* phidd) + g*sin(theta).*thetad);
        ineq = [ineq norm_energy_dot - sigma(i)];

    end
     
    eq = [];
end