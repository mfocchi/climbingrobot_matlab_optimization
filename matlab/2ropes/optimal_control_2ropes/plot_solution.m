function plot_solution(solution,p0, pf,Fleg_max,Fr_max,mu, params)

    figure
    x(1:3) = solution.Fleg;
    x(4) =solution.Tf;
    x(params.num_params+1:params.num_params+params.N_dyn) =solution.Fr_l;
    x(params.num_params+params.N_dyn+1:params.num_params+2*params.N_dyn)=solution.Fr_r;
    [c ceq, num_constr, solution_constr] = constraints(x, p0(:), pf(:),  Fleg_max, Fr_max, mu, params);
    plot_curve( solution,solution_constr, p0(:), pf(:), mu,  false, 'r', true, params);

    
    figure
    subplot(2,1,1)           
    plot(solution.time, solution.Fr_l, 'ko-'); grid on;hold on;  ylabel('Frl'); grid on;hold on;
    subplot(2,1,2)   
    plot(solution.time, solution.Fr_r, 'bo-'); grid on;hold on;  ylabel('Frr'); grid on;hold on;

    
end