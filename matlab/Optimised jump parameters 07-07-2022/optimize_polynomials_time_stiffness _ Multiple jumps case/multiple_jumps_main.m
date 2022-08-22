%
clear all ; close all ; clc
tic
global m g w1 w2 w3 w4 w5 p0 pf N num_params Fun_max mu x_vec0  x_vecf y_vec0  y_vecf z_vec0  z_vecf n_jumps k dt l_uncompressed 

m = 5;
g = 9.81;

n_jumps = 3 ;

for k= 1:n_jumps
    
% physical limits
Fun_max =20;
mu = 0.5;
% tol = 0.1;


DER_ENERGY_CONSTRAINT = true;
w1 = 1 ; % green initial cost (not used)
w2 = 1; %red final cost (not used)
w3 = 1 ; % energy weight E
w4 = 10.0; % slacks initial / final 
w5 = 0.01; %ekin0

N = 10 ; % energy constraints

dt=0.001;
num_params = 1+12+1 ;

% % Multiple jumps out of the wall (x different than 0)
%       x_vec0 = [0.1 0.4 1] ; y_vec0 = [-0.2 0 0.3] ; z_vec0 = [-1 -1.1 -1.2]; %  Points chosen out of the wall (x=0)
%       x_vecf = [0.4 1 0]    ; y_vecf = [0 0.3 0.6]  ; z_vecf = [-1.1 -1.2 -1.25];
 
% % Multiple jumps on the wall (x=0)
  x_vec0 = [0.01 0 0]  ; y_vec0 = [-2 -0.5 1] ; z_vec0 = [-0.7 -1.5 -2.5]; %  Points chosen in the wall (x=0) 
  x_vecf = [0 0 0]  ; y_vecf = [-0.5 1 2.1]    ; z_vecf = [-1.5 -2.5 -3];   %  where pf is lower than p0 (go DOWN)


%  x_vec0 = [0.01 0 0]  ; y_vec0 = [-2 -0.5 0.5] ; z_vec0 = [-1.7 -2.3 -2.7]; %  Points chosen in the wall (x=0) 
%  x_vecf = [0 0 0]  ; y_vecf = [-0.5 0.5 2]    ; z_vecf = [-2.3 -2.7 -4];   %  where pf is lower than p0 (go DOWN)

%  x_vec0 = [0.01 0 0.001]  ; y_vec0 = [-2 -0.5 1] ; z_vec0 = [-1.7 -2.3 -3]; %  Points chosen in the wall (x=0) 
%  x_vecf = [0 0.001 0.001]  ; y_vecf = [-0.5 1 2.3]    ; z_vecf = [-2.3 -3 -3.2];   %  where pf is lower than p0 (go DOWN)

%    x_vec0 = [0.01 0.01 0.001]  ; y_vec0 = [-2 -1 0] ; z_vec0 = [-0.2 -2 -3]; %  Points chosen in the wall (x=0) 
%    x_vecf = [0.01 0.001 0.01]  ; y_vecf = [-1 0 1.5]    ; z_vecf = [-2 -3 -3.5];   %  where pf is lower than p0 (go DOWN)

%   x_vec0 = [0 0 0]  ; y_vec0 = [-2.3 -1.5 -0.5] ; z_vec0 = [-1.2 -2.3 -2.7]; %  Points chosen in the wall (x=0) 
%   x_vecf = [0 0 0.001]  ; y_vecf = [-1.5 -0.5 0.6]    ; z_vecf = [-2.3 -2.7 -3.2];   %  where pf is lower than p0 (go DOWN)

theta0 = 0.05; %theta0 = 0.523
phi0 = 0 ;
l_0 = 3;
l_uncompressed = l_0;
% l_uncompressed = l_0*cos(theta0);
thetaf= 0.4 ;
phif = 1.5468 ;

%pendulum period
T_pend = 2*pi*sqrt(l_0/g)/4; % half period
% p0 = [l_0*sin(theta0)*cos(phi0); l_0*sin(theta0)*sin(phi0); -l_0*cos(theta0)];
% pf = [0.001; 5; -8];

%       p0 = [l0(k)*sin(theta0(k))*cos(phi0(k)); l0(k)*sin(theta0(k))*sin(phi0(k)); -l0(k)*cos(theta0(k))];
%       pf = [lf(k)*sin(thetaf(k))*cos(phif(k)); l0(k)*sin(thetaf(k))*sin(phif(k)); -lf(k)*cos(thetaf(k))];
  
 p0 = [x_vec0(k);  y_vec0(k); z_vec0(k)];
 pf = [x_vecf(k);  y_vecf(k); z_vecf(k)]; 

% more meaninguful init
params0 = [ T_pend, theta0, 0.01, 0, 0,  ...
                    phi0 , 0.01, 0 ,0, ...
                    l_0, 0.01 ,0, 0, ...
                    6 ];
%params0 = 0.1*ones(1,num_params);
x0 = [params0, zeros(1,N), 0,0,0] ;
lb = [0.01,     -10*ones(1,8), -30*ones(1,4), 0.1,  zeros(1,N),    0 , 0, 0];
ub = [T_pend*2, 10*ones(1,8),  30*ones(1,4), 20,  100*ones(1,N), 100 , 100, 100 ];

% options = optimoptions('fmincon','Display','iter','Algorithm','sqp');

options = optimoptions('fmincon','Display','iter','Algorithm','sqp',  ... % does not always satisfy bounds
                        'MaxFunctionEvaluations', 10000, 'ConstraintTolerance', 1e-4);
                   
[x, final_cost, EXITFLAG, output] = fmincon(@(x) cost_mj(x, p0,  pf),x0,[],[],[],[],lb,ub,@(x) constraints_mj(x, p0,  pf, DER_ENERGY_CONSTRAINT), options);

% slacks = sum(x(num_params+1:N));

slacks_energy = x(num_params+1:num_params+N);
slacks_energy_cost = sum(slacks_energy);
slacks_initial_final = x(num_params+N+1:end);
slacks_initial_final_cost = sum(slacks_initial_final);

% evaluate constraint violation 
[c ceq, energy_constraints,wall_constraints,  force_constraints, initial_final_constraints] = constraints_mj(x, p0,  pf, DER_ENERGY_CONSTRAINT);

[p, E, path_length , initial_error , final_error ] = eval_solution_mj(x, dt,  p0, pf) ;

energy = E;
opt_Tf = x(1)

toc
plot_curve_mj( p, p0, pf,  E.Etot, false, 'k');
[Fun , Fut] = evaluate_initial_impulse_mj(x);
% low_cost = abs(final_cost )<= tol;
problem_solved = (EXITFLAG == 1) || (EXITFLAG ==2) ;


number_of_converged_solutions = nan;
opt_kin_energy = nan;
if  problem_solved 
    number_of_converged_solutions = 1;    
   
    opt_kin_energy = energy.Ekin0;% 
    opt_wasted =  energy.Ekinf;
    opt_Fut = Fut;
    opt_Fun = Fun;
    plot_curve_mj( p ,  p0, pf,    E.Etot, true, 'r'); % converged are red

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
        
end 