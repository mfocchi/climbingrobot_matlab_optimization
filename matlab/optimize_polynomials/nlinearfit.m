


%
clear all ; close all ; clc
global m l g  w1 w2 w3 p0 pf N time OLD_FORMULATION POLY_TYPE num_params
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

w1 = 1 ; % green initial
w2 = 0.6; %red final
w3 = 0.01 ;
N = 10 ; % energy constraints

index_converged = [];
index_feasible = [];
friction_violation = [];
actuation_violation = [];
unilater_violation = [];
cost_violation = [];

for i=1:length(Tf_vec)
    Tf = Tf_vec(i);

    
        %0.9872    1.0136    1.1720
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
    thetaf= 0.4 ;
    phif = 1.5468 ;

    p0 = [l*sin(theta0)*cos(phi0); l*sin(theta0)*sin(phi0); -l*cos(theta0)];
    pf = [l*sin(thetaf)*cos(phif); l*sin(thetaf)*sin(phif); -l*cos(thetaf)];
    
    % more meaninguful init
    params0 = [ theta0, 0.5, 0,0,  phi0 , 0.5, 0 ,0];
    %params0 = 0.1*ones(1,num_params);
    x0 = [params0, zeros(1,N)] ;
    lb = [-35*ones(1,num_params), zeros(1,N)];
    ub = [35*ones(1,num_params), 10*ones(1,N)];

    options = optimoptions('fmincon','Display','none','Algorithm','sqp');
    %options =
    %optimoptions('fmincon','Display','none','Algorithm','interior-point',
    %'MaxIterations', 1500); %bigger slacks and cost
   
    
    [x, final_cost, EXITFLAG] = fmincon(@(x) cost(x, l, p0,  pf,  time),x0,[],[],[],[],lb,ub,@(x) constraints(x, l, time), options);
    slacks(i) = sum(x(num_params+1:end));
    
    [p(i,:,:), E, path_length(i), initial_error(i), final_error(i)] = eval_solution(x, Tf,dt, l, p0, pf);  
   
    energy(i) = E; 
    x_vec(i,:) = x;
    plot_curve(l, squeeze(p(i,:,:)), p0, pf,  E.Etot, false, 'k');
    


    [Fun , Fut] = evaluate_initial_impulse(x, 0.0, l);
    Fun_vec(i) = Fun;
    Fut_vec(i) = Fut;
    final_cost_vec(i) = final_cost;    
    low_cost = abs(final_cost )<= tol;
    problem_solved = (EXITFLAG == 1) || (EXITFLAG ==2) ;

    
    
     if (~ ( low_cost))
          cost_violation = [cost_violation i];
     end    
     if  problem_solved && low_cost
         plot_curve(l, squeeze(p(i,:,:)),  p0, pf,    E.Etot, false, 'r'); % converged are red
         index_converged = [index_converged i];
         
          % evaluate constraints on converged solutions
         actuation_constr = Fun <=  Fun_max;
         friction_constr = abs(Fut) <=  mu*Fun_max ;
         unilat_constr = Fun >=0 ;

         if  ~friction_constr
             friction_violation = [friction_violation i];
         end

         if (~  actuation_constr)
             actuation_violation = [actuation_violation i];
         end

         if (~ ( unilat_constr))
             unilater_violation = [unilater_violation i];

         end   
         
         if  actuation_constr &&  unilat_constr &&friction_constr
             index_feasible = [index_feasible i];
             plot_curve(l, squeeze(p(i,:,:)),  p0, pf,    E.Etot, false, 'g'); % feasible are green
         end
     end

end



% path length optim
[minpath_value, index_min] = min(path_length(index_feasible));
%LO energy optim
kin_energy = [energy(:).Ekin0];% + [energy(:).Ekinf];
[min_kin_energy, index_min] = min(kin_energy(index_feasible))


figure
subplot(4,1,1)
plot(Tf_vec, path_length);grid on; hold on;
plot(Tf_vec(index_feasible), path_length(index_feasible),'g.', 'MarkerSize',40);grid on;
plot(Tf_vec(index_feasible(index_min)), path_length(index_feasible(index_min)),'m.', 'MarkerSize',40);grid on;
ylabel('path_length')

subplot(4,1,2)
plot(Tf_vec, Fun_vec); hold on;grid on;
plot(Tf_vec, Fun_max*ones(1, length(Tf_vec)),'ro'); 
plot(Tf_vec, 0*ones(1, length(Tf_vec)),'bo'); 
plot(Tf_vec(index_converged), Fun_vec(index_converged),'r.', 'MarkerSize',40);grid on;
plot(Tf_vec(index_feasible), Fun_vec(index_feasible),'g.', 'MarkerSize',40);grid on;
plot(Tf_vec(index_feasible(index_min)), Fun_vec(index_feasible(index_min)),'m.', 'MarkerSize',40);grid on;

ylabel('normal constraints')

subplot(4,1,3)
plot(Tf_vec, Fut_vec); hold on;grid on;
plot(Tf_vec, mu*Fun_max*ones(1, length(Tf_vec)),'ro'); 
plot(Tf_vec, -mu*Fun_max*ones(1, length(Tf_vec)),'ro'); 

plot(Tf_vec(index_converged), Fut_vec(index_converged),'r.', 'MarkerSize',40);grid on;
plot(Tf_vec(index_feasible), Fut_vec(index_feasible),'g.', 'MarkerSize',40);grid on;
plot(Tf_vec(index_feasible(index_min)), Fut_vec(index_feasible(index_min)),'m.', 'MarkerSize',40);grid on;

ylabel('tg constraints')
subplot(4,1,4)
plot(Tf_vec, log(final_cost_vec)); hold on;grid on;
plot(Tf_vec(index_feasible), log(final_cost_vec(index_feasible)),'g.', 'MarkerSize',40);grid on;
plot(Tf_vec(index_feasible(index_min)), log(final_cost_vec(index_feasible(index_min))),'m.', 'MarkerSize',40);grid on;
plot(Tf_vec, log(tol*ones(1, length(Tf_vec))),'ro'); 
ylabel('cost')



% figure
% subplot(4,1,1)
% Ekin0x = [energy(:).Ekin0x];
% Ekin0y = [energy(:).Ekin0y];
% Ekin0z = [energy(:).Ekin0z];
% plot(Tf_vec, Ekin0x,'ro-');grid on;hold on;
% plot(Tf_vec, Ekin0y,'go-');grid on;
% plot(Tf_vec, Ekin0z,'bo-');grid on;
% plot(Tf_vec(index_feasible), Ekin0x(index_feasible),'g.', 'MarkerSize',40);grid on;
% plot(Tf_vec(index_feasible(index_min)), Ekin0x(index_feasible(index_min)),'m.', 'MarkerSize',40);grid on;
% ylabel('Ek0')
% 
% subplot(4,1,2)
% U0 = [energy(:).U0];
% plot(Tf_vec, U0,'r-');grid on;hold on;
% plot(Tf_vec(index_feasible), U0(index_feasible),'g.', 'MarkerSize',40);grid on;
% plot(Tf_vec(index_feasible(index_min)), U0(index_feasible(index_min)),'m.', 'MarkerSize',40);grid on;
% ylabel('U0')
% 
% subplot(4,1,3)
% Ekinfx = [energy(:).Ekinfx];
% Ekinfy = [energy(:).Ekinfy];
% Ekinfz = [energy(:).Ekinfz];
% plot(Tf_vec, Ekinfx,'ro-');grid on;hold on;
% plot(Tf_vec, Ekinfy,'go-');grid on;
% plot(Tf_vec, Ekinfz,'bo-');grid on;
% plot(Tf_vec(index_feasible), Ekinfx(index_feasible),'g.', 'MarkerSize',40);grid on;
% plot(Tf_vec(index_feasible(index_min)), Ekinfx(index_feasible(index_min)),'m.', 'MarkerSize',40);grid on;
% ylabel('Ekf')
% 
% subplot(4,1,4)
% Uf = [energy(:).Uf];
% plot(Tf_vec, [energy(:).Uf],'ro-');grid on;hold on;
% plot(Tf_vec(index_feasible), Uf(index_feasible),'g.', 'MarkerSize',40);grid on;
% plot(Tf_vec(index_feasible(index_min)), Ekinfx(index_feasible(index_min)),'m.', 'MarkerSize',40);grid on;
% ylabel('Uf')


% number_of_maxforce_violation = length(actuation_violation)
% number_of_unilat_violation = length(unilater_violation)
% number_of_friction_violation = length(friction_violation)
%number_of_cost_violation = length(cost_violation)
number_of_converged_solutions = length(index_converged)
number_of_feasible_solutions = length(index_feasible)
index_not_feasible = [1:N_search];
index_not_feasible(index_feasible)=[];

if number_of_feasible_solutions >0
    figure
    opt_Tf = Tf_vec(index_feasible(index_min));
    opt_position = squeeze(p(index_feasible(index_min),:,:));
    opt_E = energy(index_feasible(index_min));
    plot_curve(l, opt_position,p0, pf, opt_E.Etot, true, 'm') ; % optimal is magenta
    
%     opt_E.Ekin0x    
%     opt_E.Ekin0y    
%     opt_E.Ekin0z
%     
%     opt_E.Ekinfx    
%     opt_E.Ekinfy    
%     opt_E.Ekinfz
    
    %checks
%     (opt_E.Ekin0x + opt_E.Ekin0y + opt_E.Ekin0z + opt_E.U0) - mean(opt_E.Etot)
%     (opt_E.Ekinfx + opt_E.Ekinfy + opt_E.Ekinfz + opt_E.Uf) - mean(opt_E.Etot)
    
    
    
end

 


