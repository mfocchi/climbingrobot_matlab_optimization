%
clear all ; close all ; clc
global m  g w1 w2 w3 w4 w5 N   num_params  l_uncompressed 

m = 5;
g = 9.81;


% physical limits
Fun_max =15;
Fr_max =80; % Fr in negative
mu = 0.8;

w1 = 1 ; % green initial cost (not used)
w2 = 1; %red final cost (not used)
w3 = 1 ; % energy weight E
w4 = 10.0; % slacks initial / final 
w5 = 0.01; %ekin0

N = 10 ; % energy constraints

dt=0.001;
num_params = 1+12+1; % time + poly + K 

% Marco Frego test: initial state
theta0 = 0.05; 
phi0 = 0 ;
l_0 = 3;
p0 = [l_0*sin(theta0)*cos(phi0); l_0*sin(theta0)*sin(phi0); -l_0*cos(theta0)];
% Marco Frego test: final state
pf = [0.001; 5; -8];


% p0 =[        0.149937507812035;
%                          0;
%           -7.24];
   [theta0, phi0, l_0] = computePolarVariables(p0);
%  
%custom target

%pf = [0.001; 10; -19.9962507811849];

% compute final points for Marco
%[thetaf, phif, lf] = computePolarVariables(pf)

l_uncompressed = l_0;
%pendulum period
T_pend = 2*pi*sqrt(l_0/g)/4; % half period

% more meaninguful init
params0 = [ T_pend, theta0, 0.01, 0, 0,  ...
                    phi0 , 0.01, 0 ,0, ...
                    l_0, 0.01 ,0, 0, ...
                    6 ];
%params0 = 0.1*ones(1,num_params);
x0 = [params0, zeros(1,N), 0,0,0] ;
lb = [0.01,     -10*ones(1,8), -30*ones(1,4), 0.1,  zeros(1,N),    0 , 0, 0];
ub = [T_pend*2, 10*ones(1,8),  30*ones(1,4), 20,  100*ones(1,N), 100 , 100, 100 ];

options = optimoptions('fmincon','Display','none','Algorithm','sqp',  ... % does not always satisfy bounds
                        'MaxFunctionEvaluations', 10000, 'ConstraintTolerance', 1e-4);

% You misunderstand the documentation: the 'sqp' and 'interior-point' algorithms satisfy BOUNDS at all iterations, not nonlinear constraints.
% For your problem, it is clear that the merit function that the solvers use internally is not working well. 
% This merit function balances the two goals of lowering infeasibility and lowering the objective function. 
% Therefore, I suggest that you artificially make your constraint function much larger, to have fmincon pay more attention to it. 
% Multiply mycon by 1e4 or so (that is, have mycon return a value that is a large factor multiplied by the true constraint value)and see whether that helps.

% options check available options in:
%optimset fmincon  or 
% optimoptions('fmincon','Algorithm', 'sqp') or
%https://it.mathworks.com/help/optim/ug/output-function.html

tic
[x, final_cost, EXITFLAG, output] = fmincon(@(x) cost(x, p0,  pf),x0,[],[],[],[],lb,ub,@(x)  constraints(x, p0,  pf, Fun_max, Fr_max, mu), options);
toc

slacks_energy = x(num_params+1:num_params+N);
slacks_energy_cost = sum(slacks_energy);
slacks_initial_final = x(num_params+N+1:end);
slacks_initial_final_cost = sum(slacks_initial_final);

% evaluate constraint violation 
[c ceq, energy_constraints,wall_constraints, retraction_force_constraints, force_constraints, initial_final_constraints] = constraints(x, p0,  pf,  Fun_max, Fr_max, mu);

%output.constrviolation % your solution is infeasible! (should converge to zero; e.g. 1e-8)
%output.firstorderopt % the first-order optimality measure is the infinity norm (meaning maximum absolute value) of the gradient and Should converge to zero too (e.g. 1e-8)! Not achieved in your example!

[p, theta, phi, l, E, path_length , initial_error , final_error ] = eval_solution(x, dt,  p0, pf) ;

energy = E;
opt_Tf = x(1)
opt_K = x(14)

plot_curve( p, p0, pf,  E.Etot, false, 'k');
[Fun , Fut] = evaluate_initial_impulse(x);
problem_solved = (EXITFLAG == 1) || (EXITFLAG ==2) ;


number_of_converged_solutions = nan;
initial_kin_energy = nan;
final_kin_energy = nan;
if  problem_solved 
    number_of_converged_solutions = 1;       
    initial_kin_energy = energy.Ekin0;% 
    final_kin_energy =  energy.Ekinf;
    opt_Fut = Fut;
    opt_Fun = Fun;
    plot_curve( p ,  p0, pf,    E.Etot, true, 'r'); % converged are red

end

    
number_of_converged_solutions
initial_kin_energy
final_kin_energy
energy.intEkin
Fun
Fut 
path_length
initial_error
final_error


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
plot(-opt_K*(l-l_uncompressed)); hold on; grid on;
plot(0*ones(size(l)),'r');
plot(-Fr_max*ones(size(l)),'r');


figure
plot(energy.Ekin); hold on; grid on;
ylabel('Ekin')

%[number_of_converged_solutions,  initial_kin_energy,  final_kin_energy,  opt_Fun, opt_Fut, opt_K, opt_Tf, T_pend,  solve_time] = eval_jump(pf, Fun_max, Fr_max, mu)
 