clear all ; close all ; clc


addpath('../optimize_polynomials');
global m  g  w1 w2  w3  N  OLD_FORMULATION POLY_TYPE num_params dt tol Fun_max mu w1_mult w2_mult



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

w1_mult = 1;
w2_mult = 0.1; 
% 
target_des = [0 10, -15];
thetaf = atan2(target_des(2), -target_des(3));
lf = norm(target_des);

Njumps = 3;
lp_vec0 = ones(1,Njumps)*lf/Njumps;
theta_vec0 = ones(1,Njumps)*thetaf/Njumps;

%int_target_des = compute_intermediate_targets(target_des, Njumps);

% more meaninguful init
x0 = [lp_vec0, theta_vec0] ;
lb = [zeros(1,Njumps), zeros(1,Njumps)];
ub = [15*ones(1,Njumps),1.3*ones(1,Njumps)];


options = optimoptions('fmincon','Display','iter','Algorithm','sqp');
[x, final_cost, EXITFLAG] = fmincon(@(x) cost_multiple(x, target_des, Njumps),x0,[],[],[],[],lb,ub,@(x) constraints_multiple(x, Njumps), options);
%[number_of_feasible_solutions,number_of_converged_solutions,  opt_kin_energy,  opt_wasted, opt_Fun, opt_Fut] = eval_jump(2, 0.1,0.03, dt, tol, Fun_max, mu);       

