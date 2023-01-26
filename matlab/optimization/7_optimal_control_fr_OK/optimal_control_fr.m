clear all ; close all ; clc
global m  g w1 w2 w3 w4 w5 w6 num_params   T_th N_dyn FRICTION_CONE SUBSTEP_INTEGRATION int_method

m = 5;
g = 9.81;

% physical limits
Fun_max = 1000;
Fr_max = 200; % Fr in negative
mu = 0.8;
T_th = 0.025;


FRICTION_CONE = 1; %0
TIME_OPTIMIZATION = 1;
SUBSTEP_INTEGRATION = 1;
int_steps = 10; %5
%int_method = 'euler';
int_method = 'rk4';

w1 = 1 ; % green initial cost (not used)
w2 = 1; %red final cost (not used)
w3 = 1 ; % slacks energy weight E
w4 = 0.1; % diff Fr
w5 = 1; %ekinf (important! energy has much higher values!)
w6 = 0.1; %Fr
N_dyn = 20; %dynamic constraints (discretization)

dt=0.001; % to evaluate solution

if FRICTION_CONE
    N_dyn = 20;
end

% INITIAL STATE
l_0 = 3;
theta0 =atan2(0.38, l_0);
%theta0 = 0.05; 
phi0 = 0 ;
p0 = [l_0*sin(theta0)*cos(phi0); l_0*sin(theta0)*sin(phi0); -l_0*cos(theta0)];

%FINAL TARGET
pf=  [3.00; 3.00; -20.00]; 


%pendulum period
T_pend = 2*pi*sqrt(l_0/g)/4; % half period TODO replace with linearized
constr_tolerance = 1e-4;
options = optimoptions('fmincon','Display','iter','Algorithm','sqp',  ... % does not always satisfy bounds
'MaxFunctionEvaluations', 10000, 'ConstraintTolerance', constr_tolerance);

num_params = 3;    
x0 = [  0,    0.0,         T_pend ,      0*ones(1,N_dyn)]; %opt vars=   thetad0, phid0, K,/time, slacks_dyn, slacks_energy,   sigma =    %norm(p_f - pf)
lb = [ -30,   -30,           0.01, -Fr_max*ones(1,N_dyn)];
ub = [  30,    30,             inf, 0*ones(1,N_dyn)];
tic
[x, final_cost, EXITFLAG, output] = fmincon(@(x) cost(x, p0,  pf, int_steps),x0,[],[],[],[],lb,ub,@(x)  constraints(x, p0,  pf, Fun_max, Fr_max, mu, int_steps), options);
toc
% evaluate constraint violation 
[c ceq, num_constr, solution_constr] = constraints(x, p0,  pf,  Fun_max, Fr_max, mu, int_steps);
solution = eval_solution(x, dt,  p0, pf) ;
solution.cost = final_cost;
problem_solved = (EXITFLAG == 1) || (EXITFLAG == 2);
% EXITFLAG ==1 First-order optimality measure was less than options.OptimalityTolerance, and maximum constraint violation was less than options.ConstraintTolerance.
% EXITFLAG == 2 Change in x was less than options.StepTolerance and maximum constraint violation was less than options.ConstraintTolerance.
%EXITFLAG == 0 max number of iterations

if problem_solved
    plot_curve( solution,solution_constr, p0, pf, mu,  false, 'r', true);
else 
    fprintf(2,"Problem didnt converge!\n")
    plot_curve( solution,solution_constr, p0, pf, mu,  false, 'k', true);
end
opt_Tf = solution.time(end)
 
fprintf('Fun:  %f\n\n',solution.Fun)
fprintf('Fut:  %f\n\n',solution.Fut)
fprintf('cost:  %f\n\n',solution.cost)
fprintf('final_kin_energy:  %f\n\n',solution.energy.Ekinf)
fprintf('initial_error:  %f\n\n',solution.initial_error)
fprintf('final_error_real:  %f\n\n',solution.final_error_real)
fprintf('final_error_discrete:  %f\n\n', solution_constr.final_error_discrete)
fprintf('max_integration_error:  %f\n\n', solution.final_error_real - solution_constr.final_error_discrete)

%for Daniele
save('test_matlab_1.mat','solution','T_th','mu','Fun_max', 'Fr_max', 'p0','pf');



DEBUG = false;

if (DEBUG)
    eval_constraints(c, num_constr, constr_tolerance)  

    figure
    ylabel('Fr')
    plot(solution.time,0*ones(size(solution.l)),'k'); hold on; grid on;
    plot(solution.time,-Fr_max*ones(size(solution.l)),'k');
    plot(solution_constr.time,solution.Fr_rough,'bo'); hold on; grid on;   
    plot(solution.time,solution.Fr,'r');
      
    figure
    subplot(3,1,1)
    plot(solution.time, solution.theta,'r');hold on; grid on;
    plot(solution_constr.time, solution_constr.theta,'ob');
    ylabel('theta')

    subplot(3,1,2)
    plot(solution.time, solution.phi,'r');hold on; grid on;
    plot(solution_constr.time, solution_constr.phi,'ob');
    ylabel('phi')
    
    subplot(3,1,3)
    plot(solution.time, solution.l,'r'); hold on; grid on;
    plot(solution_constr.time, solution_constr.l,'ob');
    ylabel('l')
    
    figure
    subplot(3,1,1)
    plot(solution.time, solution.thetad,'r');hold on; grid on;
    plot(solution_constr.time, solution_constr.thetad,'ob');
    ylabel('thetad')

    subplot(3,1,2)
    plot(solution.time, solution.phid,'r');hold on; grid on;
    plot(solution_constr.time, solution_constr.phid,'ob');
    ylabel('phid')
    
    subplot(3,1,3)
    plot(solution.time, solution.ld,'r'); hold on; grid on;
    plot(solution_constr.time, solution_constr.ld,'ob');
    ylabel('ld')
        
    
    figure
    subplot(3,1,1)
    plot(solution.time, solution.p(1,:),'r') ; hold on;    
    plot(solution_constr.time, solution_constr.p(1,:),'ob') ; hold on;    
    ylabel('X')
    
    subplot(3,1,2)
    plot(solution.time, solution.p(2,:),'r') ; hold on;    
    plot(solution_constr.time, solution_constr.p(2,:),'ob') ; hold on;    
    ylabel('Y')
    
    subplot(3,1,3)
    plot(solution.time, solution.p(3,:),'r') ; hold on;    
    plot(solution_constr.time, solution_constr.p(3,:),'ob') ; hold on;
    ylabel('Z')
       
    
end

disp('inputs from optimization')
p0 
solution.Fun
solution.Fut
disp('check')
opt_Tf
solution.achieved_target


