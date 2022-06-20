


%
clear all ; close all ; clc
global m l g  w1 w2 w3 p0 pf N time OLD_FORMULATION POLY_TYPE num_params
m = 5;
g = 9.81;

% physical limits
Fun_max = 30 ;
mu = 0.5;
tol = .1;

w1 = 1 ; % green initial
w2 = 0.6; %red final
w3 = 0.01 ;
N = 10 ; % energy constraints

dt=0.001;
OLD_FORMULATION = 1;
POLY_TYPE = 0; % 0 cubic, 1 quintic

if (POLY_TYPE)
    num_params = 12;
else 
    num_params = 8; 
end



l_range = [2:1:10];
thetaf_range = [0.1:0.1:0.9];

lthetaf_vector = [];
feasible =[];
converged = [];
initial_kin_energy = [];
final_kin_energy = [];

for k=1:length(l_range)    
    for j=1:length(thetaf_range)       

        l = l_range(k);
        thetaf= thetaf_range(j);
        %pendulum period
        T_pend = 2*pi*sqrt(l/g)/2; % half period
        N_search = 20;
        Tf_vec=linspace(0.5*T_pend, 1.5*T_pend, N_search);
        
        
        index_converged = [];
        index_feasible = [];

        friction_violation = [];
        actuation_violation = [];
        unilater_violation = [];
        cost_violation = [];
        
        for i=1:length(Tf_vec)
            Tf = Tf_vec(i);
            
            time = linspace(0, Tf, N) ;
            theta0 = 0.03; %theta0 = 0.523
            phi0 = 0 ;    
            phif = 1.5468 ;
           
        
            
            p0 = [l*sin(theta0)*cos(phi0); l*sin(theta0)*sin(phi0); -l*cos(theta0)];
            pf = [l*sin(thetaf)*cos(phif); l*sin(thetaf)*sin(phif); -l*cos(thetaf)];
            % more meaninguful init
            params0 = [ theta0, 0.5, 0,0,  phi0 , 0.5, 0 ,0];
            %params0 = 0.1*ones(1,num_params);
            x0 = [params0, zeros(1,N)] ;
            lb = [-35*ones(1,num_params), zeros(1,N)];
            ub = [35*ones(1,num_params), 10*ones(1,N)];

            %options = optimoptions('fmincon','Display','iter','Algorithm','sqp');
            options = optimoptions('fmincon','Display','none','Algorithm','sqp', 'MaxIterations', 1500);

            [x, final_cost, EXITFLAG] = fmincon(@cost,x0,[],[],[],[],lb,ub,@constraints, options);
            slacks(i) = sum(x(num_params+1:end));


            
            [p(i,:,:), E, path_length(i)] = eval_solution(x, Tf,dt);

            energy(i) = E; 
            poly_coeff(i,:) = x;

            [Fun , Fut] = evaluate_initial_impulse(x, 0.0);
            Fun_vec(i) = Fun;
            Fut_vec(i) = Fut;
            final_cost_vec(i) = final_cost;    
            low_cost = abs(final_cost )<= tol;
            problem_solved = (EXITFLAG == 1) || (EXITFLAG ==2) ;
    
            if (~ ( low_cost))
                cost_violation = [cost_violation i];
            end
            if  problem_solved && low_cost
                index_converged = [index_converged i];
                
                % evaluate constraints on converged solutions
                actuation_constr = Fun <=  Fun_max;
                friction_constr = abs(Fut) <=  mu*Fun_max ;
                unilat_constr = Fun >=0 ;
                
                if  ~friction_constr
                    friction_violation = [friction_violation i];
                end
                
                if (~  actuation_constr)
                    actuation_violation = [actuation_violation i];
                end
                
                if (~ ( unilat_constr))
                    unilater_violation = [unilater_violation i];
                    
                end
                
                if  actuation_constr &&  unilat_constr &&friction_constr
                    index_feasible = [index_feasible i];
                    
                end
            end
        end 

        % path length optim
        %[minvalue, index_min] = min(path_length(index_feasible))
        %LO energy optim
        kin_energy = [energy(:).Ekin0];% 
        wasted_energy =  [energy(:).Ekinf];
        [opt_kin_energy, index_min] = min(kin_energy(index_feasible));
        %eval the force for the optimal choice
        opt_Fut = Fut_vec(index_feasible(index_min));
        opt_Fun = Fun_vec(index_feasible(index_min));
        opt_wasted = wasted_energy(index_feasible(index_min));
        
              
        %number_of_maxforce_violation = length(actuation_violation)
        %number_of_unilat_violation = length(unilater_violation)
        %number_of_friction_violation = length(friction_violation)
        %number_of_cost_violation = length(cost_violation)
        number_of_converged_solutions = length(index_converged);
        if length(index_converged) == 0
            number_of_converged_solutions = nan;
        end
        number_of_feasible_solutions = length(index_feasible);
        if length(index_feasible) == 0
            number_of_feasible_solutions = nan;
        else
            plot_curve(squeeze(p(index_feasible(index_min),:,:)),  p0, pf,    E.Etot, false, 'm'); 
        end
        
        if  isempty(opt_kin_energy) % no solution was found
            opt_kin_energy = nan;
        end
        if  isempty(opt_wasted) % no solution was found
            opt_wasted = nan;
        end
        if  isempty(opt_Fut) % no solution was found
            opt_Fut = nan;
        end
        if  isempty(opt_Fun) % no solution was found
            opt_Fun = nan;
        end
        
      
        lthetaf_vector = [lthetaf_vector [l ; thetaf]];
        feasible = [feasible number_of_feasible_solutions];    
        converged = [converged number_of_converged_solutions];
        initial_kin_energy = [initial_kin_energy opt_kin_energy] ;
        final_kin_energy = [final_kin_energy opt_wasted] ;
     
        
        fprintf('l =%3.2f    thetaf =%5.2f    feas=%5d    conv=%5d    Ekin0=%5.2f   Ekinf = %5.2f    Fun=%5.2f   Fut=%5.2f \n',...
                 l, thetaf, number_of_feasible_solutions, number_of_converged_solutions, opt_kin_energy,  opt_wasted, opt_Fun, opt_Fut);

    end
end
    
l_range_dense = [l_range(1):0.01:l_range(end)];
thetaf_range_dense = [thetaf_range(1):0.005:thetaf_range(end)];

plot_surf("feasible", l_range_dense, thetaf_range_dense, lthetaf_vector(1,:) ,   lthetaf_vector(2,:),    feasible);
plot_surf("conv",     l_range_dense, thetaf_range_dense, lthetaf_vector(1,:) ,   lthetaf_vector(2,:),    converged);
plot_surf("initial kin energy",l_range_dense, thetaf_range_dense, lthetaf_vector(1,:) ,   lthetaf_vector(2,:),    initial_kin_energy);
plot_surf("final kin energy",l_range_dense, thetaf_range_dense, lthetaf_vector(1,:) ,   lthetaf_vector(2,:),    final_kin_energy);



save('lookuptables.mat', 'lthetaf_vector', 'initial_kin_energy', 'final_kin_energy', 'feasible', 'converged', 'l_range_dense', 'thetaf_range_dense');
  