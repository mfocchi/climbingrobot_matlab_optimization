clear all ; close all ; clc
global m  g w1 w2 w3 w4 w5 w6 N num_params  l_uncompressed T_th N_dyn FRICTION_CONE 

m = 5;
g = 9.81;

% physical limits
Fun_max =1000;
Fr_max =130; % Fr in negative
mu = 2.0;
T_th = 0.05;
FRICTION_CONE=0;
TIME_OPTIMIZATION = 1;

w1 = 1 ; % green initial cost (not used)
w2 = 1; %red final cost (not used)
w3 = 1 ; % slacks energy weight E
w4 = 100.0; % slacks  final (used)
w5 = 0.001; %ekinf (important! energy has much higher values!)
w6 = 0.0001; %slacks dynamics

N = 10 ; % energy constraints
N_dyn = 40; %dynamic constraints (discretization)

dt=0.001; % to evaluate solution


% Marco Frego test: initial state
l_0 = 3;
theta0 =atan2(0.38, l_0);
%theta0 = 0.05; 
phi0 = 0 ;
p0 = [l_0*sin(theta0)*cos(phi0); l_0*sin(theta0)*sin(phi0); -l_0*cos(theta0)];

% Marco Frego test: final state
pf = [1.0; 5.0; -8];

l_uncompressed = l_0;
%pendulum period
T_pend = 2*pi*sqrt(l_0/g)/4; % half period TODO replace with linearized

constr_tolerance = 1e-4;

%test
%[states, t] = integrate_dynamics([theta0; phi0; l_0; 0;0;0], dt_dyn, N_dyn,10)

if TIME_OPTIMIZATION
    options = optimoptions('fmincon','Display','iter','Algorithm','sqp',  ... % does not always satisfy bounds
    'MaxFunctionEvaluations', 10000, 'ConstraintTolerance', constr_tolerance);

    num_params = 4;    
    x0 = [  0, 0.0,     6,     T_pend,      zeros(1,N),    zeros(1,N_dyn),           0]; %opt vars=   thetad0, phid0, K,/time, slacks_dyn, slacks_energy,   sigma =    %norm(p_f - pf)
    % with thetad0 = 1 it detaches from the wall but does not reach the
    % target
    %lb = [ 1,    -30,   0.1,    0.01,         zeros(1,N),     zeros(1,N_dyn),       0,  0];
    lb = [ -30,    -30,   0.1,    0.01,        zeros(1,N),     zeros(1,N_dyn),         0];
    ub = [  30,   30,    40,   T_pend*2,      100*ones(1,N),   100*ones(1,N_dyn),    100];
    [x, final_cost, EXITFLAG, output] = fmincon(@(x) cost(x, p0,  pf),x0,[],[],[],[],lb,ub,@(x)  constraints(x, p0,  pf, Fun_max, Fr_max, mu), options);
    % evaluate constraint violation 
    [c ceq, num_constr, solution_constr] = constraints(x, p0,  pf,  Fun_max, Fr_max, mu);
    solution = eval_solution(x, dt,  p0, pf) ;
    solution.cost = final_cost;
    problem_solved = (EXITFLAG == 1) || (EXITFLAG == 2);
    % EXITFLAG ==1 First-order optimality measure was less than options.OptimalityTolerance, and maximum constraint violation was less than options.ConstraintTolerance.
    % EXITFLAG == 2 Change in x was less than options.StepTolerance and maximum constraint violation was less than options.ConstraintTolerance.
    
    if problem_solved
        plot_curve( solution,solution_constr, p0, pf,  true, 'r');
    else 
        fprintf(2,"Problem didnt converge!")
        plot_curve( solution,solution_constr, p0, pf,  true, 'k');
    end
else
    
    fprintf(2, 'MAIN LOOP: time optim off, batch search\n')
    options = optimoptions('fmincon','Display','none','Algorithm','sqp',  ... % does not always satisfy bounds
    'MaxFunctionEvaluations', 10000, 'ConstraintTolerance', constr_tolerance);

    N_search = 10;
    Tf =linspace(0.5*T_pend, 2*T_pend, N_search);
    min_final_error = 100;
    optimal_traj_index =1;
    solution_constr_vec=[];
    solution_vec = [];
    figure
    for i=1:N_search
        num_params = 3;    
        x0 = [  0,     0.0,    6,           zeros(1,N),    zeros(1,N_dyn),           0]; %opt vars=   thetad0, phid0, K,/time, slacks_dyn, slacks_energy,   sigma =    %norm(p_f - pf)
        lb = [ -30,    -30,   0.1,          zeros(1,N),     zeros(1,N_dyn),         0];
        ub = [  30,   30,     40,        100*ones(1,N),   100*ones(1,N_dyn),      100];
        [x, final_cost, EXITFLAG, output] = fmincon(@(x) cost(x, p0,  pf, Tf(i)),x0,[],[],[],[],lb,ub,@(x)  constraints(x, p0,  pf, Fun_max, Fr_max, mu, Tf(i)), options);
        % evaluate constraint violation 
        [c ceq,num_constr, solution_constr] = constraints(x, p0,  pf,  Fun_max, Fr_max, mu, Tf(i));
        solution = eval_solution(x, dt,  p0, pf, Tf(i)) ;
        solution.cost = final_cost;
        problem_solved = (EXITFLAG == 1) || (EXITFLAG == 2);
           
        if problem_solved
            if solution_constr.final_error_discrete < min_final_error
                min_final_error = solution_constr.final_error_discrete;
                optimal_traj_index = i;                
            else 
                plot_curve( solution,solution_constr, p0, pf, false, 'r');
            end
        else 
            plot_curve( solution,solution_constr, p0, pf, false, 'k');            
        end
        solution_vec  = [solution_vec solution];
        solution_constr_vec = [solution_constr_vec solution_constr];
    end
    fprintf(2, 'bath search RESULTS\n')
    optimal_traj_index
    back_search_Tf = Tf(optimal_traj_index)
    solution = solution_vec(optimal_traj_index);
    solution_constr = solution_constr_vec(optimal_traj_index);
    plot_curve( solution,solution_constr, p0, pf, false, 'g');  
end

opt_K = solution.K;
opt_Tf = solution.time(end);

 
fprintf('Fun:  %f\n\n',solution.Fun)
fprintf('Fut:  %f\n\n',solution.Fut)
fprintf('cost:  %f\n\n',solution.cost)
fprintf('final_kin_energy:  %f\n\n',solution.energy.Ekinf)
fprintf('initial_error:  %f\n\n',solution.initial_error)
fprintf('final_error_real:  %f\n\n',solution.final_error_real)
fprintf('final_error_discrete:  %f\n\n', solution_constr.final_error_discrete)
fprintf(strcat('slacks_energy: ', repmat(' %f ', 1, N),' \n\n'),solution.slacks_energy)
fprintf(strcat('slacks_dyn: ', repmat(' %f ', 1, N_dyn),' \n\n'),solution.slacks_dyn)
fprintf('slacks_final:  %f\n\n',solution.slacks_initial_final)

%for Daniele
save('test.mat','solution','T_th','mu','Fun_max', 'Fr_max', 'p0','pf');

DEBUG = true;

if (DEBUG)

    if any( c(1:num_constr.energy_constraints)>constr_tolerance)
            disp('1- energy constraints')
        c(1:num_constr.energy_constraints)
    end

    wall_constraints_idx = num_constr.energy_constraints;    
    if any(c(wall_constraints_idx+1:wall_constraints_idx+num_constr.wall_constraints)>constr_tolerance)
        disp('2- wall constraints')
        c(wall_constraints_idx+1:wall_constraints_idx  + num_constr.wall_constraints)
    end  
    
    
    retraction_force_constraints_idx = wall_constraints_idx+num_constr.wall_constraints;
    
    if any( c(retraction_force_constraints_idx+1:retraction_force_constraints_idx+num_constr.retraction_force_constraints)>constr_tolerance)
        disp('3- retraction force constraints')
         c(retraction_force_constraints_idx+1:retraction_force_constraints_idx + num_constr.retraction_force_constraints)
    end

    
    

    force_constraints_idx = retraction_force_constraints_idx + num_constr.retraction_force_constraints;
    
    if any(c(force_constraints_idx+1: force_constraints_idx + num_constr.force_constraints)>constr_tolerance)
        disp('4 -force constraints')
        c(force_constraints_idx+1: force_constraints_idx + num_constr.force_constraints)
        Fun
        Fut
    end
    

   
    init_dynamic_constraints_idx = force_constraints_idx + num_constr.force_constraints ;
    if  any(c(init_dynamic_constraints_idx+1: init_dynamic_constraints_idx+num_constr.dynamic_constraints)>constr_tolerance)
        disp('5 - dynamics  constraints')
        c(init_dynamic_constraints_idx+1: init_dynamic_constraints_idx + num_constr.dynamic_constraints)
    end
    
    
    init_final_constraints_idx = init_dynamic_constraints_idx+num_constr.dynamic_constraints;
    if  any(   c(init_final_constraints_idx+1: init_final_constraints_idx+num_constr.initial_final_constraints)>constr_tolerance)
        disp('6 - initial final  constraints')
        c(init_final_constraints_idx+1: init_final_constraints_idx+num_constr.initial_final_constraints)
    end
    
 


    figure
    ylabel('Fr')
    plot(solution.time,solution.Fr_vec); hold on; grid on;
    plot(solution.time,0*ones(size(solution.l)),'r');
    plot(solution.time,-Fr_max*ones(size(solution.l)),'r');


%     figure
%     subplot(2,1,1)
%     plot(solution.p(1,:))
%     ylabel('X')
% 
%     subplot(2,1,2)
%     plot(solution.ld)
%     ylabel('ld')
%     
%     figure
%     plot(time, solution.energy.Ekin); hold on; grid on;
%     ylabel('Ekin')
%     
   
%     figure
%     subplot(3,1,1)
%     plot(solution.time, solution.theta,'r');hold on; grid on;
%     plot(solution_constr.time, solution_constr.theta,'-ob');
%     ylabel('theta')
% 
%     subplot(3,1,2)
%     plot(solution.time, solution.phi,'r');hold on; grid on;
%     plot(solution_constr.time, solution_constr.phi,'-ob');
%     ylabel('phi')
%     
%     subplot(3,1,3)
%     plot(solution.time, solution.l,'r'); hold on; grid on;
%     plot(solution_constr.time, solution_constr.l,'-ob');
%     ylabel('l')
%     
%     figure
%     subplot(3,1,1)
%     plot(solution.time, solution.thetad,'r');hold on; grid on;
%     plot(solution_constr.time, solution_constr.thetad,'-ob');
%     ylabel('thetad')
% 
%     subplot(3,1,2)
%     plot(solution.time, solution.phid,'r');hold on; grid on;
%     plot(solution_constr.time, solution_constr.phid,'-ob');
%     ylabel('phid')
%     
%     subplot(3,1,3)
%     plot(solution.time, solution.ld,'r'); hold on; grid on;
%     plot(solution_constr.time, solution_constr.ld,'-ob');
%     ylabel('ld')
%     
%     
    
    figure
    subplot(3,1,1)
    plot(solution.time, solution.p(1,:),'r') ; hold on;    
    plot(solution_constr.time, solution_constr.p(1,:),'-ob') ; hold on;    
    ylabel('X')
    
    subplot(3,1,2)
    plot(solution.time, solution.p(2,:),'r') ; hold on;    
    plot(solution_constr.time, solution_constr.p(2,:),'-ob') ; hold on;    
    ylabel('Y')
    
    subplot(3,1,3)
    plot(solution.time, solution.p(3,:),'r') ; hold on;    
    plot(solution_constr.time, solution_constr.p(3,:),'-ob') ; hold on;
    ylabel('Z')
    
end

disp('inputs')
p0
solution.Fut 
solution.Fun
opt_K
disp('check')
opt_Tf


