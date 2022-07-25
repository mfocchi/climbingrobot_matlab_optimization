


%
clear all ; close all ; clc
global m l g  w1 w2 w3 w4 N   num_params
m = 5;
l = 4;
g = 9.81;

%pendulum period
T_pend = 2*pi*sqrt(l/g)/4; % half period
N_search = 20;
Tf_vec=linspace(0.5*T_pend, 1.5*T_pend, N_search);
%Tf_vec=linspace(0.1, 1, N_search);

% physical limits
Fun_max = 20;
mu = 0.5;
tol = .1;

DER_ENERGY_CONSTRAINT = true;
w1 = 1 ; % green initial
w2 = 0.6; %red final
if DER_ENERGY_CONSTRAINT
    w3 = 0.01 ; % energy weight dE
else
    w3 = 0.0001 ; % energy weight E
end
w4 = 0.00005; % energy weight cost Ekin0

N = 10 ; % energy constraints

index_converged = [];
index_feasible = [];
friction_violation = [];
actuation_violation = [];
unilater_violation = [];
cost_violation = [];


dt=0.001;
num_params = 8+1;


theta0 = 0.05; %theta0 = 0.523
phi0 = 0 ;
thetaf= 0.4 ;
phif = 1.5468 ;

p0 = [l*sin(theta0)*cos(phi0); l*sin(theta0)*sin(phi0); -l*cos(theta0)];
pf = [l*sin(thetaf)*cos(phif); l*sin(thetaf)*sin(phif); -l*cos(thetaf)];

% more meaninguful init
params0 = [ T_pend, theta0, 0.5, 0,0,  phi0 , 0.5, 0 ,0];
%params0 = 0.1*ones(1,num_params);
x0 = [params0, zeros(1,N)] ;
lb = [0.0, -35*ones(1,num_params-1), zeros(1,N)];
ub = [T_pend*5, 35*ones(1,num_params-1), 10*ones(1,N)];

options = optimoptions('fmincon','Display','none','Algorithm','sqp',  ... % does not always satisfy bounds
                        'MaxFunctionEvaluations', 10000, 'ConstraintTolerance', 1e-4);


[x, final_cost, EXITFLAG] = fmincon(@(x) cost(x, l, p0,  pf),x0,[],[],[],[],lb,ub,@(x) constraints(x, l, DER_ENERGY_CONSTRAINT), options);
slacks = sum(x(num_params+1:end));

[p, E, path_length , initial_error , final_error ] = eval_solution(x, x(1),dt, l, p0, pf) ;

energy = E;
opt_Tf = x(1)


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
    plot_curve(l,  p ,  p0, pf,    E.Etot, true, 'r'); % converged are red
    % evaluate constraints on converged solutions
    actuation_constr = sqrt(Fun^2 + Fut^2) <=  Fun_max;
    friction_constr = abs(Fut) <=  mu*Fun;
    unilat_constr = Fun >=0;

    opt_kin_energy = energy.Ekin0;% 
    opt_wasted =  energy.Ekinf;
    opt_Fut = Fut;
    opt_Fun = Fun;

    if  unilat_constr && friction_constr && unilat_constr
        number_of_feasible_solutions = 1;

    end

end

opt_kin_energy
number_of_feasible_solutions
number_of_converged_solutions
initial_error
final_error
%[number_of_feasible_solutions,number_of_converged_solutions,  opt_kin_energy,  opt_wasted, opt_Fun, opt_Fut, opt_Tf] = eval_jump(l , thetaf , theta0, dt, Fun_max, mu, DER_ENERGY_CONSTRAINT)
        

