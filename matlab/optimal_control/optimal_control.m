

clear all ; close all ; clc
global m  g w1 w2 w3 w4 w5 N num_params  l_uncompressed T_th N_dyn dt_dyn

m = 5;
g = 9.81;


% physical limits
Fun_max =150;
Fr_max =80; % Fr in negative
mu = 0.8;
T_th = 0.05;

w1 = 1 ; % green initial cost (not used)
w2 = 1; %red final cost (not used)
w3 = 1 ; % energy weight E
w4 = 10.0; % slacks initial / final 
w5 = 0.01; %ekinf (important! energy has much higher values!)

N = 10 ; % energy constraints

dt=0.001;
dt_dyn = 0.02;


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
N_dyn = floor(T_pend/dt_dyn);

num_params = 3;
%opt vars=   thetad0, phid0, K, slacks_dyn, slacks_energy,   sigma =
%norm(p_f - pf)  /time
x0 = [   0,    0,    1,    zeros(1,N),    0];%, T_pend ];
lb = [0.01,    0,  -10,    zeros(1,N)     0];% ,0.01];
ub = [  10,   10,   20, 100*ones(1,N),  100];%, T_pend*2, ];

%test
[states, t] = integrate_dynamics([theta0; phi0; l_0; 0;0;0], dt_dyn, N_dyn,10)

options = optimoptions('fmincon','Display','iter','Algorithm','sqp',  ... % does not always satisfy bounds
                        'MaxFunctionEvaluations', 10000, 'ConstraintTolerance', 1e-2);

tic
[x, final_cost, EXITFLAG, output] = fmincon(@(x) cost(x, p0,  pf),x0,[],[],[],[],lb,ub,@(x)  constraints(x, p0,  pf, Fun_max, Fr_max, mu), options);
toc

slacks_energy = x(num_params+1:num_params+N);
slacks_energy_cost = sum(slacks_energy);
slacks_initial_final = x(num_params+N+1:end);
slacks_initial_final_cost = sum(slacks_initial_final);

% evaluate constraint violation 
[c ceq, energy_constraints,wall_constraints, retraction_force_constraints, force_constraints, initial_final_constraints] = constraints(x, p0,  pf,  Fun_max, Fr_max, mu);

[p, theta, phi, l,  E, path_length , initial_error , final_error, thetad, phid,ld, time ] = eval_solution(x, dt,  p0, pf) ;

energy = E;
opt_Tf = x(1);
opt_K = x(4);

plot_curve( p, p0, pf,  E.Etot, false, 'k');
%[Fun , Fut] = evaluate_initial_impulse(x);
problem_solved = (EXITFLAG == 1) || (EXITFLAG ==2) ;


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
    disp('1- energy constraints')
    c(1:energy_constraints)

    disp('2- wall constraints')
    wall_constraints_idx = energy_constraints;
    c(wall_constraints_idx+1:wall_constraints_idx+wall_constraints)

    disp('3- retraction force constraints')
    retraction_force_constraints_idx = wall_constraints_idx+wall_constraints;
    c(retraction_force_constraints_idx+1:retraction_force_constraints_idx+retraction_force_constraints)

    disp('4 -force constraints')
    force_constraints_idx = retraction_force_constraints_idx+retraction_force_constraints;
    c(force_constraints_idx+1: force_constraints_idx+force_constraints)

    disp('5 - initial final  constraints')
    init_final_constraints_idx = force_constraints_idx+force_constraints;
    c(init_final_constraints_idx+1: init_final_constraints_idx+initial_final_constraints)

    slacks_energy 
    slacks_initial_final 


    figure
    ylabel('Fr')
    plot(Fr_vec); hold on; grid on;
    plot(0*ones(size(l)),'r');
    plot(-Fr_max*ones(size(l)),'r');


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
    plot(time, theta);hold on; grid on;
    ylabel('theta')

    subplot(3,1,2)
    plot(time, phi);hold on; grid on;
    ylabel('phi')
    
        subplot(3,1,3)
    plot(time, l); hold on; grid on;
    ylabel('l')
    
    
    
end

disp('inputs')
p0
Fut 
Fun
opt_K
disp('check')
opt_Tf
pf


