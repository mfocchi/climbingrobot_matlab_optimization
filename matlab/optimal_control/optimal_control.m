clear all ; close all ; clc
global m  g w1 w2 w3 w4 w5 w6 N num_params  l_uncompressed T_th N_dyn 

m = 5;
g = 9.81;

% physical limits
Fun_max =550;
Fr_max =180; % Fr in negative
mu = 2.8;
T_th = 0.05;

w1 = 1 ; % green initial cost (not used)
w2 = 1; %red final cost (not used)
w3 = 0.01 ; % energy weight E
w4 = 100000.0; % slacks  final (used)
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
pf = [0.001; 5; -8];

l_uncompressed = l_0;
%pendulum period
T_pend = 2*pi*sqrt(l_0/g)/2; % half period TODO replace with linearized


num_params = 4;
%opt vars=   thetad0, phid0, K,/time, slacks_dyn, slacks_energy,   sigma =
%norm(p_f - pf)  
x0 = [  0.1, 0.1,     15,     T_pend,      zeros(1,N),    zeros(1,N_dyn),        0,  0];
lb = [ -30.0,  -30,   15,    0.01,         zeros(1,N),     zeros(1,N_dyn),       0,  0];
ub = [  30,   30,    40,   T_pend*4,      100*ones(1,N),   100*ones(1,N_dyn), 100,100];
constr_tolerance = 1e-4;
%test
%[states, t] = integrate_dynamics([theta0; phi0; l_0; 0;0;0], dt_dyn, N_dyn,10)

options = optimoptions('fmincon','Display','iter','Algorithm','sqp',  ... % does not always satisfy bounds
                        'MaxFunctionEvaluations', 10000, 'ConstraintTolerance', constr_tolerance);

tic
[x, final_cost, EXITFLAG, output] = fmincon(@(x) cost(x, p0,  pf),x0,[],[],[],[],lb,ub,@(x)  constraints(x, p0,  pf, Fun_max, Fr_max, mu), options);
toc

slacks_energy = x(num_params+1:num_params+N);
slacks_energy_cost = sum(slacks_energy);
slacks_dyn = x(num_params+N+1:num_params+N+N_dyn);
slacks_initial_final = x(num_params+N+N_dyn+1:end);
slacks_initial_final_cost = sum(slacks_initial_final);

% evaluate constraint violation 
[c ceq, energy_constraints,wall_constraints, retraction_force_constraints, force_constraints, initial_final_constraints, dynamic_constraints, solution_constr] = constraints(x, p0,  pf,  Fun_max, Fr_max, mu);

[p, theta, phi, l,  E, path_length , initial_error , final_error, thetad, phid,ld, time ] = eval_solution(x, dt,  p0, pf) ;

energy = E;

opt_K = x(3);
opt_Tf = x(4);
%evaluate inpulse ( the integral of the gaussian is 1) 
Fun = m*l_0*thetad(1)/T_th;
Fut = m*l_0*sin(theta(1))*phid(1)/T_th;

plot_curve( p, p0, pf,  E.Etot, false, 'k');
%[Fun , Fut] = evaluate_initial_impulse(x);
problem_solved = (EXITFLAG == 1) ;
% EXITFLAG ==1 First-order optimality measure was less than options.OptimalityTolerance, and maximum constraint violation was less than options.ConstraintTolerance.
% EXITFLAG == 2 Change in x was less than options.StepTolerance and maximum constraint violation was less than options.ConstraintTolerance.


number_of_converged_solutions = nan;
initial_kin_energy = nan;
final_kin_energy = nan;
if  problem_solved 
    number_of_converged_solutions = 1;       
    initial_kin_energy = energy.Ekin0;% 
    final_kin_energy =  energy.Ekinf;

    plot_curve( p ,  p0, pf,    E.Etot, true, 'r'); % converged are red
end


    
number_of_converged_solutions
initial_kin_energy
final_kin_energy
Fun
Fut 
initial_error
final_error

%for Daniele
Fr_vec = -opt_K*(l-l_uncompressed);
save('test.mat','energy','Fr_vec','theta', 'phi', 'l','thetad','phid','ld','time')

DEBUG = true;

if (DEBUG)

    if any( c(1:energy_constraints)>constr_tolerance)
            disp('1- energy constraints')
        c(1:energy_constraints)
    end


    wall_constraints_idx = energy_constraints;
    if any(c(wall_constraints_idx+1:wall_constraints_idx+wall_constraints)>constr_tolerance)
        disp('2- wall constraints')
        c(wall_constraints_idx+1:wall_constraints_idx+wall_constraints)
    end
   
    
    
    retraction_force_constraints_idx = wall_constraints_idx+wall_constraints;
    if any( c(retraction_force_constraints_idx+1:retraction_force_constraints_idx+retraction_force_constraints)>constr_tolerance)
        disp('3- retraction force constraints')
         c(retraction_force_constraints_idx+1:retraction_force_constraints_idx+retraction_force_constraints)
    end


    force_constraints_idx = retraction_force_constraints_idx+retraction_force_constraints;
    if any(c(force_constraints_idx+1: force_constraints_idx+force_constraints)>constr_tolerance)
        disp('4 -force constraints')
        c(force_constraints_idx+1: force_constraints_idx+force_constraints)
        Fun
        Fut
    end
    

   
    init_dynamic_constraints_idx = force_constraints_idx+force_constraints ;
    if  any(c(init_dynamic_constraints_idx+1: init_dynamic_constraints_idx+dynamic_constraints)>constr_tolerance)
        disp('5 - dynamics  constraints')
        c(init_dynamic_constraints_idx+1: init_dynamic_constraints_idx+dynamic_constraints)
    end
    
    
    init_final_constraints_idx = init_dynamic_constraints_idx+dynamic_constraints;
    if  any(   c(init_final_constraints_idx+1: init_final_constraints_idx+initial_final_constraints)>constr_tolerance)
        disp('6 - initial final  constraints')
        c(init_final_constraints_idx+1: init_final_constraints_idx+initial_final_constraints)
    end
    
    slacks_energy 
    slacks_dyn    
    slacks_initial_final 


    figure
    ylabel('Fr')
    plot(time,Fr_vec); hold on; grid on;
    plot(time,0*ones(size(l)),'r');
    plot(time,-Fr_max*ones(size(l)),'r');


%     figure
%     subplot(2,1,1)
%     plot(p(1,:))
%     ylabel('X')
% 
%     subplot(2,1,2)
%     plot(ld)
%     ylabel('ld')
%     
%     figure
%     plot(time, energy.Ekin); hold on; grid on;
%     ylabel('Ekin')
%     
   
    figure
    subplot(3,1,1)
    plot(time, theta,'r');hold on; grid on;
    plot(solution_constr.time, solution_constr.theta,'-ob');
    ylabel('theta')

    subplot(3,1,2)
    plot(time, phi,'r');hold on; grid on;
    plot(solution_constr.time, solution_constr.phi,'-ob');
    ylabel('phi')
    
    subplot(3,1,3)
    plot(time, l,'r'); hold on; grid on;
    plot(solution_constr.time, solution_constr.l,'-ob');
    ylabel('l')
    
    
    
    figure
    subplot(3,1,1)
    plot(time, p(1,:),'r') ; hold on;    
    plot(solution_constr.time, solution_constr.p(1,:),'-ob') ; hold on;    
    ylabel('X')
    
    subplot(3,1,2)
    plot(time, p(2,:),'r') ; hold on;    
    plot(solution_constr.time, solution_constr.p(2,:),'-ob') ; hold on;    
    ylabel('Y')
    
    subplot(3,1,3)
    plot(time, p(3,:),'r') ; hold on;    
    plot(solution_constr.time, solution_constr.p(3,:),'-ob') ; hold on;
    ylabel('Z')
    
    
    
    
    
end

disp('inputs')
p0
Fut 
Fun
opt_K
disp('check')
opt_Tf
pf


