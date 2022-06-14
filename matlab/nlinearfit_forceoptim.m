
%
clear all ; close all ; clc
global m l g w1 w2 w3 p0 pf  mu N dt Fun_max theta0 phi0 num_params impulse_mean time Tf
m = 1;
l = 4;
g = 9.81;


%pendulum period
T_pend = 2*pi*sqrt(l/g)/4; % half period
N_search = 20;
Tf_vec=linspace(0.8*T_pend, 1.3*T_pend, N_search);


% physical limits
impulse_mean = 0.1;
Fun_max = 15;
mu = 0.4;
tol = 0.1;

num_params = 2;
w1 = 0.0; % minimize path length
w2 = 1; %red final
w3 = 0.01 ; %minimize energy derivative
N = 10 ; % energy constraints

index_constraints = [];
cost_violation = [];
% 


for i=1:length(Tf_vec)
    Tf = Tf_vec(i);
    dt=0.01;     
    
    time = linspace(0, Tf, Tf/dt) ;
    
    theta0 = 0.05; %theta0 = 0.523
    phi0 = 0 ;
    thetaf= 0.8864 ;
    phif = 1.5468 ;

    p0 = [l*sin(theta0)*cos(phi0); l*sin(theta0)*sin(phi0); -l*cos(theta0)];
    pf = [l*sin(thetaf)*cos(phif); l*sin(thetaf)*sin(phif); -l*cos(thetaf)];

    x0 = [[  5,       4   ], zeros(1,N)];
    lb = [[  0 ,      -10 ], zeros(1,N)]; %slacks should be biggere than zero
    ub = [[   Fun_max,  10], 10*ones(1,N)];

    %options = optimoptions('fmincon','Display','iter','Algorithm','sqp');
    options = optimoptions('fmincon','Display','none','Algorithm','sqp', 'MaxIterations', 1500);

    [x, final_cost, EXITFLAG] = fmincon(@cost_force,x0,[],[],[],[],lb,ub,@constraints_force, options);
    slacks = sum(x(num_params:end));

    [E, path_length(i)] = plot_curve_forceoptim(x, p0, pf,false, false);

   
    energy(i) =mean(E); 
    x_vec(i,:) = x;
    
    % evaluate constraints 
    low_cost = abs(final_cost )<= tol;
    problem_solved = (EXITFLAG == 1) || (EXITFLAG ==2) ;
    final_cost_vec(i) = final_cost;  
    
    if (~ ( low_cost))
         cost_violation = [cost_violation i];
    end
    
    if   problem_solved && low_cost
        plot_curve_forceoptim(x, p0, pf,false, true);
        index_constraints = [index_constraints i]
       
    end
    
    
    
    
end 

[min, index_min] = min(path_length(index_constraints));
plot_results_force(Tf_vec, path_length, energy, x_vec, final_cost_vec, tol);
    
opt_Tf = Tf_vec(index_constraints(index_min))


number_of_cost_violation = length(cost_violation)
number_of_solutions = length(index_constraints)

if number_of_solutions >0
    figure
    plot_curve_forceoptim(poly_coeff(index_constraints(index_min),:), Tf_vec(index_constraints(index_min)), p0, pf,dt, true, true);
    energy(index_constraints(index_min))
    ft = x(2)*[sin(phi0), cos(phi0), 0]';
    fu = x(1)*[cos(phi0)*cos(theta0) , sin(phi0)*cos(theta0), sin(theta0)]';
    F = ft + fu;

    arrow3(p0, F/norm(F));
end

 
