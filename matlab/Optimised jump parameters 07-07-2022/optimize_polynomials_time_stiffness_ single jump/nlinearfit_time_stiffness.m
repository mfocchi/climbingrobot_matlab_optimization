%
clear all ; close all ; clc
global m  g w1 w2 w3 w4 w5 N   num_params Fun_max mu l_uncompressed 

m = 5;
g = 9.81;


% physical limits
Fun_max =20;
mu = 0.5;

DER_ENERGY_CONSTRAINT = true;
w1 = 1 ; % green initial cost (not used)
w2 = 1; %red final cost (not used)
w3 = 1 ; % energy weight E
w4 = 10.0; % slacks initial / final 
w5 = 0.01; %ekin0

N = 10 ; % energy constraints

dt=0.001;
num_params = 1+12+1;


theta0 = 0.05; %theta0 = 0.523
phi0 = 0 ;
l_0 = 3;
l_uncompressed = l_0;
% l_uncompressed = l_0*cos(theta0);
thetaf= 0.4 ;
phif = 1.5468 ;

%pendulum period
T_pend = 2*pi*sqrt(l_0/g)/4; % half period
p0 = [l_0*sin(theta0)*cos(phi0); l_0*sin(theta0)*sin(phi0); -l_0*cos(theta0)];
pf = [0.001; 5; -8];



% more meaninguful init
params0 = [ T_pend, theta0, 0.01, 0, 0,  ...
                    phi0 , 0.01, 0 ,0, ...
                    l_0, 0.01 ,0, 0, ...
                    6 ];
%params0 = 0.1*ones(1,num_params);
x0 = [params0, zeros(1,N), 0,0,0] ;
lb = [0.01,     -10*ones(1,8), -30*ones(1,4), 0.1,  zeros(1,N),    0 , 0, 0];
ub = [T_pend*2, 10*ones(1,8),  30*ones(1,4), 20,  100*ones(1,N), 100 , 100, 100 ];

options = optimoptions('fmincon','Display','iter','Algorithm','sqp',  ... % does not always satisfy bounds
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
[x, final_cost, EXITFLAG, output] = fmincon(@(x) cost(x, p0,  pf),x0,[],[],[],[],lb,ub,@(x)  constraints(x, p0,  pf, DER_ENERGY_CONSTRAINT), options);

slacks_energy = x(num_params+1:num_params+N);
slacks_energy_cost = sum(slacks_energy);
slacks_initial_final = x(num_params+N+1:end);
slacks_initial_final_cost = sum(slacks_initial_final);

% evaluate constraint violation 
[c ceq, energy_constraints,wall_constraints,  force_constraints, initial_final_constraints] = constraints(x, p0,  pf, DER_ENERGY_CONSTRAINT);

%output.constrviolation % your solution is infeasible! (should converge to zero; e.g. 1e-8)
%output.firstorderopt % the first-order optimality measure is the infinity norm (meaning maximum absolute value) of the gradient and Should converge to zero too (e.g. 1e-8)! Not achieved in your example!

[p, E, path_length , initial_error , final_error ] = eval_solution(x, dt,  p0, pf) ;

energy = E;
opt_Tf = x(1)


plot_curve( p, p0, pf,  E.Etot, false, 'k');
[Fun , Fut] = evaluate_initial_impulse(x);
problem_solved = (EXITFLAG == 1) || (EXITFLAG ==2) ;


number_of_converged_solutions = nan;
opt_kin_energy = nan;
if  problem_solved 
    number_of_converged_solutions = 1;    
   
    opt_kin_energy = energy.Ekin0;% 
    opt_wasted =  energy.Ekinf;
    opt_Fut = Fut;
    opt_Fun = Fun;
    plot_curve( p ,  p0, pf,    E.Etot, true, 'r'); % converged are red

end


    
number_of_converged_solutions
opt_kin_energy
Fun
Fut 
initial_error
final_error
opt_K = x(14)

disp('1- energy constraints')
c(1:energy_constraints)

disp('2- wall constraints')
wall_constraints_idx = energy_constraints;
c(wall_constraints_idx+1:wall_constraints_idx+wall_constraints)


disp('4 -force constraints')
force_constraints_idx = wall_constraints_idx+wall_constraints;
c(force_constraints_idx+1: force_constraints_idx+force_constraints)

disp('5 - initial final  constraints')
init_final_constraints_idx = force_constraints_idx+force_constraints;
c(init_final_constraints_idx+1: init_final_constraints_idx+initial_final_constraints)

slacks_energy 
slacks_initial_final 

%[number_of_feasible_solutions,number_of_converged_solutions,  opt_kin_energy,  opt_wasted, opt_Fun, opt_Fut, opt_Tf] = eval_jump(l , thetaf , theta0, dt, Fun_max, mu, DER_ENERGY_CONSTRAINT)
        
