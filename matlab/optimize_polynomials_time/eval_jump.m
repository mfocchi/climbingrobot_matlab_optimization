function [number_of_feasible_solutions, number_of_converged_solutions, opt_kin_energy, opt_wasted, opt_Fun, opt_Fut, opt_Tf] = eval_jump(l, thetaf, theta0, dt, Fun_max, mu, DER_ENERGY_CONSTRAINT) 

        global g  w1 w2 w3 N  num_params 
        
        if ~exist('DER_ENERGY_CONSTRAINT','var')
            DER_ENERGY_CONSTRAINT = false;
        end
        
        %pendulum period
        T_pend = 2*pi*sqrt(l/g)/4; % half period
       
        
        tol = .1;
        w1 = 1 ; % green initial
        w2 = 0.6; %red final
        if DER_ENERGY_CONSTRAINT
            w3 = 0.01 ; % energy weight dE
        else
            w3 = 0.0001 ; % energy weight E
        end
        N = 10 ; % number of energy constraints

        
        index_converged = [];
        index_feasible = [];

        friction_violation = [];
        actuation_violation = [];
        unilater_violation = [];
        cost_violation = [];
       
        
        num_params = 8+1;
       
        phi0 = 0 ;
        phif = 1.5468 ;

        p0 = [l*sin(theta0)*cos(phi0); l*sin(theta0)*sin(phi0); -l*cos(theta0)];
        pf = [l*sin(thetaf)*cos(phif); l*sin(thetaf)*sin(phif); -l*cos(thetaf)];

        % more meaninguful init
        params0 = [ T_pend, theta0, 0.5, 0,0,  phi0 , 0.5, 0 ,0];
        %params0 = 0.1*ones(1,num_params);
        x0 = [params0, zeros(1,N)] ;
        lb = [0.0, -35*ones(1,num_params-1), zeros(1,N)];
        ub = [T_pend*5, 35*ones(1,num_params-1), 10*ones(1,N)];

        options = optimoptions('fmincon','Display','none','Algorithm','sqp');
        %options =
        %optimoptions('fmincon','Display','none','Algorithm','interior-point',
        %'MaxIterations', 1500); %bigger slacks and cost


        [x, final_cost, EXITFLAG] = fmincon(@(x) cost(x, l, p0,  pf),x0,[],[],[],[],lb,ub,@(x) constraints(x, l, DER_ENERGY_CONSTRAINT), options);
        slacks = sum(x(num_params+1:end));

        [p, E, path_length , initial_error , final_error ] = eval_solution(x, x(1),dt, l, p0, pf) ;

        energy = E;
        opt_Tf = x(1);

        
        plot_curve(l, p, p0, pf,  E.Etot, false, 'k');
        [Fun , Fut] = evaluate_initial_impulse(x, 0.0, l);
        low_cost = abs(final_cost )<= tol;
        problem_solved = (EXITFLAG == 1) || (EXITFLAG ==2) ;

        number_of_feasible_solutions = nan;
        number_of_converged_solutions = nan;
        opt_kin_energy = nan;% 
        opt_wasted =  nan;
        opt_Fut = nan;
        opt_Fun = nan;

        
        if  problem_solved 
            number_of_converged_solutions = 1;
            plot_curve(l,  p ,  p0, pf,    E.Etot, false, 'm'); % optimal is magenta
            % evaluate constraints on converged solutions
            actuation_constr = Fun <=  Fun_max;
            friction_constr = abs(Fut) <=  mu*Fun_max;
            unilat_constr = Fun >=0;
            
            opt_kin_energy = energy.Ekin0;% 
            opt_wasted =  energy.Ekinf;
            opt_Fut = Fut;
            opt_Fun = Fun;
            
            if  unilat_constr && friction_constr && unilat_constr
                number_of_feasible_solutions = 1;
                          
            end
           
        end
        
        
end