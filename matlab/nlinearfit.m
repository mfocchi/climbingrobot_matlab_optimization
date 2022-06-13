


%
clear all ; close all ; clc
global m l g  w1 w2 w3 p0 pf N time OLD_FORMULATION POLY_TYPE num_params
m = 1;
l = 4;
g = 9.81;

%pendulum period

T_pend = 2*pi*sqrt(l/g)/4; % half period
N_search = 20;
Tf_vec=linspace(0.8*T_pend, 1.3*T_pend, N_search);
%Tf_vec=linspace(0.1, 1, N_search);

% physical limits
Fun_max = 7;
mu = 0.5;
tol = .1;

w1 = 1 ; % green initial
w2 = 0.6; %red final
w3 = 0.01 ;
N = 10 ; % energy constraints

index_constraints = [];
friction_violation = [];
actuation_violation = [];
unilater_violation = [];
cost_violation = [];

for i=1:length(Tf_vec)
    Tf = Tf_vec(i);

    
    
% for i=1:1
%     Tf = 1.;
%     
    
    
   
    dt=0.001;


    OLD_FORMULATION = 1;
    POLY_TYPE = 0; % 0 cubic, 1 quintic

    if (POLY_TYPE)
        num_params = 12;
    else 
        num_params = 8; 
    end

    time = linspace(0, Tf, N) ;
    theta0 = 0.05; %theta0 = 0.523
    phi0 = 0 ;
    thetaf= 0.68864 ;
    phif = 1.5468 ;

    p0 = [l*sin(theta0)*cos(phi0); l*sin(theta0)*sin(phi0); -l*cos(theta0)];
    pf = [l*sin(thetaf)*cos(phif); l*sin(thetaf)*sin(phif); -l*cos(thetaf)];

    x0 = [0.1*ones(1,num_params), zeros(1,N)] ;
    lb = [-35*ones(1,num_params), zeros(1,N)];
    ub = [35*ones(1,num_params), 10*ones(1,N)];

    %options = optimoptions('fmincon','Display','iter','Algorithm','sqp');
    options = optimoptions('fmincon','Display','none','Algorithm','sqp', 'MaxIterations', 1500);

    [x, final_cost, EXITFLAG] = fmincon(@cost,x0,[],[],[],[],lb,ub,@constraints, options);
    slacks = sum(x(9:end));


    [E, path_length(i)] = plot_curve(x, Tf, p0, pf,dt, false, false);

   
    energy(i) =mean(E); 
    poly_coeff(i,:) = x;

    
    % evaluate constraints 
    [Fun , Fut] = evaluate_initial_impulse(x, 0.0);
    actuation_constr = Fun <=  Fun_max;
    friction_constr = abs(Fut) <=  mu*Fun_max ;
    unilat_constr = Fun >=0 ;
    low_cost = abs(final_cost )<= tol;
    problem_solved = (EXITFLAG == 1) || (EXITFLAG ==2) ;
    
    Fun_vec(i) = Fun;
    Fut_vec(i) = Fut;
    final_cost_vec(i) = final_cost;
    
    if  ~friction_constr
        friction_violation = [friction_violation i];
    end
    
     if (~  actuation_constr)
         actuation_violation = [actuation_violation i];
     end
    
   if (~ ( unilat_constr))
         unilater_violation = [unilater_violation i];
   end
    
      if (~ ( low_cost))
         cost_violation = [cost_violation i];
    end
    
    if  actuation_constr && friction_constr && unilat_constr   && problem_solved && low_cost
        plot_curve(x, Tf, p0, pf,dt, false, true);
        index_constraints = [index_constraints i];
       
    end

end
% 

[min, index_min] = min(path_length(index_constraints));

figure
subplot(5,1,1)
plot(Tf_vec, path_length);grid on;
ylabel('path_length')
subplot(5,1,2)
plot(Tf_vec, energy) ;grid on;
ylabel('energy')


subplot(5,1,3)
plot(Tf_vec, Fun_vec); hold on;grid on;
plot(Tf_vec, Fun_max*ones(1, length(Tf_vec)),'ro'); 
plot(Tf_vec, 0*ones(1, length(Tf_vec)),'bo'); 
ylabel('normal constraints')


subplot(5,1,4)
plot(Tf_vec, Fut_vec); hold on;grid on;
plot(Tf_vec, mu*Fun_max*ones(1, length(Tf_vec)),'ro'); 
plot(Tf_vec, -mu*Fun_max*ones(1, length(Tf_vec)),'ro'); 
ylabel('tg constraints')


subplot(5,1,5)
plot(Tf_vec, log(final_cost_vec)); hold on;grid on;
plot(Tf_vec, log(tol*ones(1, length(Tf_vec))),'ro'); 
ylabel('cost')




opt_Tf = Tf_vec(index_constraints(index_min))

number_of_friction_violation = length(friction_violation)
number_of_actuation_violation = length(actuation_violation)
number_of_unilat_violation = length(unilater_violation)
number_of_cost_violation = length(cost_violation)
number_of_solutions = length(index_constraints)

if number_of_solutions >0
    figure
    plot_curve(poly_coeff(index_constraints(index_min),:), Tf_vec(index_constraints(index_min)), p0, pf,dt, true, true);
    energy(index_constraints(index_min))
end

 


