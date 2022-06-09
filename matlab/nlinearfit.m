


%
clear all ; close all ; clc
global m l g  w1 w2 w3 p0 pf N time OLD_FORMULATION POLY_TYPE num_params
m = 1;
l = 4;
g = 9.81;
Tf_vec=1:0.05:3;

% physical limits
Fun_max = 5;
mu = 0.5;
tol = 0.008;

index_constraints = [];

for i=1:length(Tf_vec)
    Tf = Tf_vec(i);

%     
%     
% for i=1:1
%     Tf = 1.5;
    
    
    
   
    dt=0.001;

    w1 = 1 ; % green initial
    w2 = 0.6; %red final
    w3 = 0.1 ;

    N = 10 ;
    OLD_FORMULATION = 1;
    POLY_TYPE = 0; % 0 cubic, 1 quintic

    if (POLY_TYPE)
        num_params = 12;
    else 
        num_params = 8; 
    end

    time = linspace(0, Tf, N) ;
    theta0 = pi/6 ; %theta0 = 0.523
    phi0 = 0 ;
    thetaf= 0.8864 ;
    phif = 1.5468 ;

    p0 = [l*sin(theta0)*cos(phi0); l*sin(theta0)*sin(phi0); -l*cos(theta0)];
    pf = [l*sin(thetaf)*cos(phif); l*sin(thetaf)*sin(phif); -l*cos(thetaf)];

    x0 = [0.1*ones(1,num_params), zeros(1,N)] ;
    lb = [-10*ones(1,num_params), zeros(1,N)];
    ub = [10*ones(1,num_params), 10*ones(1,N)];

    options = optimoptions('fmincon','Display','iter','Algorithm','sqp');

    [x, final_cost] = fmincon(@cost,x0,[],[],[],[],lb,ub,@constraints, options)
    slacks = sum(x(9:end));


    [E, norm_pd] = plot_curve(x, Tf, p0, pf,dt, false);

    path_length(i) = sum(norm_pd*dt);
    energy(i) =mean(E); 
    poly_coeff(i,:) = x;
    
    
    % evaluate constraints 
    [Fun , Fut] = evaluate_initial_impulse( x, Tf_vec(i))
    actuation_constr = Fun <=  Fun_max;
    friction_constr = abs(Fut) <=  mu*Fun_max ;
    unilat_constr = Fun >=0 ;
    feasible_traj = abs(final_cost )<= tol
    
    Fun_vec(i) = Fun;
    Fut_vec(i) = Fut;
    final_cost_vec(i) = final_cost;
    
    
    if  actuation_constr && friction_constr && unilat_constr &&feasible_traj
      
        index_constraints = [index_constraints i]
    end
    
end
% 

[min, index_min] = min(path_length(index_constraints))

figure
subplot(5,1,1)
plot(Tf_vec, path_length);grid on;
subplot(5,1,2)
plot(Tf_vec, energy) ;grid on;
subplot(5,1,3)
plot(Tf_vec, Fun_vec); hold on;grid on;
plot(Tf_vec, Fun_max*ones(1, length(Tf_vec)),'ro'); 
plot(Tf_vec, 0*ones(1, length(Tf_vec)),'bo'); 

subplot(5,1,4)
plot(Tf_vec, Fut_vec); hold on;grid on;
plot(Tf_vec, mu*Fun_max*ones(1, length(Tf_vec)),'ro'); 
plot(Tf_vec, -mu*Fun_max*ones(1, length(Tf_vec)),'ro'); 

subplot(5,1,5)
plot(Tf_vec, log(final_cost_vec)); hold on;grid on;
plot(Tf_vec, log(tol*ones(1, length(Tf_vec))),'ro'); 


opt_Tf = Tf_vec(index_constraints(index_min))
number_of_solutions = length(index_constraints)


if number_of_solutions >0
    figure
    plot_curve(poly_coeff(index_constraints(index_min),:), opt_Tf, p0, pf,dt, false);
end

 


