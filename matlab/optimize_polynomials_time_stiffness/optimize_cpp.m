
function [number_of_converged_solutions,  initial_kin_energy,  final_kin_energy,  opt_Fun, opt_Fut, opt_K, opt_Tf] = optimize_cpp(l_0, theta0, phi0,  pf, Fun_max, Fr_max, mu) 

global m  g w1 w2 w3 w4 w5 N   num_params  l_uncompressed T_th

m = 5;
g = 9.81;
T_th = 0.05;

pf = pf(:);

w1 = 1 ; % green initial cost (not used)
w2 = 1; %red final cost (not used)
w3 = 1 ; % energy weight E
w4 = 10.0; % slacks initial / final 
w5 = 0.01; %ekinf

N = 10 ; % energy constraints

dt=0.001;
num_params = 1+12+1; % time + poly + K 

p0 = [l_0*sin(theta0)*cos(phi0); l_0*sin(theta0)*sin(phi0); -l_0*cos(theta0)];

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
tic
[x, final_cost, EXITFLAG, output] = fmincon(@(x) cost(x, p0,  pf),x0,[],[],[],[],lb,ub,@(x)  constraints(x, p0,  pf, Fun_max, Fr_max, mu), options);
toc

[p, theta, phi, l,  E, path_length , initial_error , final_error, thetad, phid,ld, time ] = eval_solution(x, dt,  p0, pf) ;

energy = E;
opt_Tf = x(1);
opt_K = x(14);

[Fun , Fut] = evaluate_initial_impulse(x);
problem_solved = (EXITFLAG == 1) || (EXITFLAG ==2) ;

number_of_converged_solutions = nan;
initial_kin_energy = nan;
final_kin_energy = nan;
opt_Fun = nan;
opt_Fut = nan;

if  problem_solved 
    number_of_converged_solutions = 1;       
    initial_kin_energy = energy.Ekin0;% 
    final_kin_energy =  energy.Ekinf;
    opt_Fut = Fut;
    opt_Fun = Fun;    
   
end


end

