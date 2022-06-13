


%
clear all ; close all ; clc
global m l g  w1 w2 w3 p0 pf mu N dt Tf Fun_max theta0 phi0 num_params
m = 1;
l = 4;
g = 9.81;
Tf_vec=0.1:0.2:5;

% physical limits
Fun_max = 5;
mu = 0.5;
tol = 0.1;

num_params = 6;
w1 = 1 ; % green initial
w2 = 0.2; %red final
w3 = 0.01 ;
N = 10 ; % energy constraints

index_constraints = [];
friction_violation = [];
actuation_violation = [];
unilater_violation = [];
cost_violation = [];
% 
% for i=1:length(Tf_vec)
%     Tf = Tf_vec(i);

    
    
for i=1:1
    Tf = 1.;
    
   
    dt=0.001;
    N = Tf/dt;
    

    time = linspace(0, Tf, N) ;
    theta0 = 0.05; %theta0 = 0.523
    phi0 = 0 ;
    thetaf= 0.8864 ;
    phif = 1.5468 ;

    p0 = [l*sin(theta0)*cos(phi0); l*sin(theta0)*sin(phi0); -l*cos(theta0)];
    pf = [l*sin(thetaf)*cos(phif); l*sin(thetaf)*sin(phif); -l*cos(thetaf)];

    x0 = [0.1*ones(1,num_params)]%, zeros(1,N)] ;
    lb = [-35*ones(1,num_params)]%, zeros(1,N)]; %slacks should be biggere than zero
    ub = [35*ones(1,num_params)]%, 10*ones(1,N)];

    %options = optimoptions('fmincon','Display','iter','Algorithm','sqp');
    options = optimoptions('fmincon','Display','none','Algorithm','sqp', 'MaxIterations', 1500);

    [x, final_cost, EXITFLAG] = fmincon(@cost_force,x0,[],[],[],[],lb,ub,@constraints_force, options);
    slacks = sum(x(9:end));


    [E, path_length(i) ] = plot_curve_forceoptim(x, time, p0, pf,dt, false, false);

   
    energy(i) =mean(E); 
    Fun_vec(i) = x(3); 
    Fut_vec(i) = x(6);

    
    % evaluate constraints 


    low_cost = abs(final_cost )<= tol;
    problem_solved = (EXITFLAG == 1) || (EXITFLAG ==2) ;
    final_cost_vec(i) = final_cost;
    
  
    
      if (~ ( low_cost))
         cost_violation = [cost_violation i];
    end
    
    if   problem_solved && low_cost
        plot_curve(x, Tf, p0, pf,dt, false, true);
        index_constraints = [index_constraints i];
       
    end

end
% 

[min, index_min] = min(path_length(index_constraints));

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

number_of_friction_violation = length(friction_violation)
number_of_actuation_violation = length(actuation_violation)
number_of_unilat_violation = length(unilater_violation)
number_of_cost_violation = length(cost_violation)
number_of_solutions = length(index_constraints)

if number_of_solutions >0
    figure
    plot_curve(poly_coeff(4,:), Tf_vec(4), p0, pf,dt, false, true);
end

 


