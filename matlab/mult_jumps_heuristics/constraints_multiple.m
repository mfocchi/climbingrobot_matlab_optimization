function [ineq, eq] = constraints_multiple(x, Njumps)
   global  dt tol Fun_max mu
   lp_vec = x(1:Njumps);
   theta_vec = x(Njumps+1:2*Njumps);
   [target, l_vec, theta_r] = compute_target(lp_vec, theta_vec);
  
   % ineq are <= 0
   abs_angle0(1) = 0.02;
   for i=1:Njumps 
        if (i>=2)
           abs_angle0(i) = sum( theta_vec(1:i-1)) - sum(theta_r(1:i));
        end
        [number_of_feasible_solutions,number_of_converged_solutions,  opt_kin_energy,  opt_wasted, opt_Fun, opt_Fut] = eval_jump(l_vec(i), theta_vec(i), abs_angle0(i), dt, tol, Fun_max, mu);
        ineq(i) = - (number_of_feasible_solutions -1 );
    end    
number_of_feasible_solutions
    eq = [];
end