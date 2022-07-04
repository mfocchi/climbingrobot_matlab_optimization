%
clear all ; close all ; clc
global m  g w1 w2 w3 w4 N   num_params Fun_max mu l_uncompressed 

m = 5;
g = 9.81;


% physical limits
Fun_max =20;
mu = 0.5;
tol = .1;

DER_ENERGY_CONSTRAINT = true;
w1 = 1 ; % green initial
w2 = 1; %red final
w3 = 0.8 ; % energy weight E
w4 = 0.001; % energy weight cost Ekin0

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
lf = 8;

%pendulum period
T_pend = 2*pi*sqrt(l_0/g)/4; % half period
p0 = [l_0*sin(theta0)*cos(phi0); l_0*sin(theta0)*sin(phi0); -l_0*cos(theta0)];
%pf = [lf*sin(thetaf)*cos(phif); lf*sin(thetaf)*sin(phif); -lf*cos(thetaf)];


pf = [0.001; 5; -10];



% more meaninguful init
params0 = [ T_pend, sin(theta0), 0.2, 0, 0,  ...
                    sin(phi0) , 0.2, 0 ,0, ...
                    l_0, 0 ,0, 0, ...
                    5 ];
%params0 = 0.1*ones(1,num_params);
x0 = [params0, zeros(1,N), 1,1] ;
lb = [0,     -10*ones(1,12),   1,  zeros(1,N), 0 , 0];
ub = [T_pend*5, 10*ones(1,12), 100,  10*ones(1,N), 10 , 10 ];

options = optimoptions('fmincon','Display','iter','Algorithm','sqp');
%options =
%optimoptions('fmincon','Display','none','Algorithm','interior-point',
%'MaxIterations', 1500); %bigger slacks and cost


[x, final_cost, EXITFLAG] = fmincon(@(x) cost(x, p0,  pf),x0,[],[],[],[],lb,ub,@(x) constraints(x, p0,  pf, DER_ENERGY_CONSTRAINT), options);
% slacks = sum(x(num_params+1:end));
slacks = sum(x(num_params+1:N));

[p, E, path_length , initial_error , final_error ] = eval_solution(x, dt,  p0, pf) ;

energy = E;
opt_Tf = x(1)


% plot_curve( p, p0, pf,  E.Etot, false, 'k');
[Fun , Fut] = evaluate_initial_impulse(x);
low_cost = abs(final_cost )<= tol;
problem_solved = (EXITFLAG == 1) || (EXITFLAG ==2) ;

% number_of_feasible_solutions = nan;
% number_of_converged_solutions = nan;
% opt_kin_energy = nan;% 
% opt_wasted =  nan;
% opt_Fut = nan;
% opt_Fun = nan;


plot_curve( p ,  p0, pf,    E.Etot, true, 'r'); % converged are red

if  problem_solved 
    number_of_converged_solutions = 1;
    
    % evaluate constraints on converged solutions
    actuation_constr = Fun <=  Fun_max;
    friction_constr = abs(Fut) <=  mu*Fun_max;
    unilat_constr = Fun >=0;

    opt_kin_energy = energy.Ekin0;% 
    opt_wasted =  energy.Ekinf;
    opt_Fut = Fut;
    opt_Fun = Fun;

    if  unilat_constr && friction_constr && actuation_constr
        number_of_feasible_solutions = 1;
         plot_curve( p ,  p0, pf,    E.Etot, true, 'r'); % converged are red
    end

end

%     if number_of_feasible_solutions > 0
%         disp('It converged')
%     else
%        
%         disp('didnot converge')
%     end

% opt_kin_energy
% number_of_feasible_solutions
% number_of_converged_solutions
% Fun
% Fut 
% initial_error
% final_error
% opt_K = x(14)


%[number_of_feasible_solutions,number_of_converged_solutions,  opt_kin_energy,  opt_wasted, opt_Fun, opt_Fut, opt_Tf] = eval_jump(l , thetaf , theta0, dt, Fun_max, mu, DER_ENERGY_CONSTRAINT)
        
