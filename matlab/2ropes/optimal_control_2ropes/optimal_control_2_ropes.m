clear all ; close all ; clc
global m  g w1 w2 w3 w4 w5 w6 num_params FRICTION_CONE m g b p_a1 p_a2 T_th N_dyn int_method contact_normal 


% physical limits
Fleg_max = 300;
Fr_max = 90; % Fr is negative
mu = 0.8;
T_th = 0.05;
contact_normal =[1;0;0];
jump_clearance = 1;

addpath('../simulation/compact_model');
%int_method = 'euler';
int_method = 'rk4';

w1 = 1 ; % green initial cost (not used)
w2 = 1; %red final cost (not used)
w3 = 1 ; % slacks energy weight E
w4 = 1; % diff Fr1/2 smothing
w5 = 1; %ekinf (important! energy has much higher values!)
w6 = 1; %Fr work
N_dyn = 50; %dynamic constraints (discretization) 200
FRICTION_CONE = 1;

%WORLD FRAME ATTACHED TO ANCHOR 1
anchor_distance = 5;
b = anchor_distance;
p_a1 = [0;0;0];
p_a2 = [0;anchor_distance;0];
g = 9.81;
m = 5.08;   % Mass [kg]

%jump params
% INITIAL POINT
p0 = [0.5; 2.5; -6]; % there is singularity for px = 0!

%FINAL TARGET
pf= [0.5; 4;-4];


%compute initial state from jump param
x0 = computeStateFromCartesian(p0);
dt=0.001; % to evaluate solution

%pendulum period
T_pend = 2*pi*sqrt(x0(2)/g)/4; % half period TODO replace with linearized x0(2) = l10
constr_tolerance = 1e-3;
options = optimoptions('fmincon','Display','iter','Algorithm','sqp',  ... % does not always satisfy bounds
'MaxFunctionEvaluations', 10000, 'ConstraintTolerance', constr_tolerance);

num_params = 4;    
Fr_l0 = 0*ones(1,N_dyn);
Fr_r0 = 0*ones(1,N_dyn);
x0 = [  100,            0.0,          0.0,        T_pend,  Fr_l0,                               Fr_r0]; %opt vars=   Flegx Flegy Flexz Tf  traj_Fr_l traj_Fr_r
lb = [  0,   -Fleg_max, -Fleg_max          0.01, -Fr_max*ones(1,N_dyn), -Fr_max*ones(1,N_dyn)];
ub = [  Fleg_max,    Fleg_max, Fleg_max,           inf,  0*ones(1,N_dyn),            0*ones(1,N_dyn)];
tic
[x, final_cost, EXITFLAG, output] = fmincon(@(x) cost(x, p0,  pf), x0,[],[],[],[],lb,ub,  @(x) constraints(x, p0,  pf, Fleg_max, Fr_max, mu, jump_clearance) , options);
toc
% evaluate constraint violation 
[c ceq, num_constr, solution_constr] = constraints(x, p0,  pf,  Fleg_max, Fr_max, mu, jump_clearance);
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
 
fprintf('Fleg:  %f %f %f \n\n',solution.Fleg(1), solution.Fleg(2), solution.Fleg(3))
fprintf('cost:  %f\n\n',solution.cost)
fprintf('final_kin_energy:  %f\n\n',solution.energy.Ekinf)
fprintf('initial_error:  %f\n\n',solution.initial_error)
fprintf('final_error_real:  %f\n\n',solution.final_error_real)
fprintf('final_error_discrete:  %f\n\n', solution_constr.final_error_discrete)
fprintf('max_integration_error:  %f\n\n', solution.final_error_real - solution_constr.final_error_discrete)


DEBUG = true;

if (DEBUG)
    eval_constraints(c, num_constr, constr_tolerance)  
    figure
    ylabel('Fr-X')
    plot(solution.time,0*ones(size(solution.Fr_l)),'k'); hold on; grid on;
    plot(solution.time,-Fr_max*ones(size(solution.Fr_l)),'k');
    plot(solution.time,solution.Fr_l,'r');
    plot(solution.time,solution.Fr_r,'b');
    legend({'min','max','Frl','Frr'});
    
        
%     
%     figure
%     subplot(3,1,1)
%     plot(solution.time, solution.psi,'r');hold on; grid on;
%     plot(solution_constr.time, solution_constr.psi,'ob');
%     ylabel('psi')
% 
%     subplot(3,1,2)
%     plot(solution.time, solution.l1,'r');hold on; grid on;
%     plot(solution_constr.time, solution_constr.l1,'ob');
%     ylabel('phi')
%     
%     subplot(3,1,3)
%     plot(solution.time, solution.l2,'r'); hold on; grid on;
%     plot(solution_constr.time, solution_constr.l2,'ob');
%     ylabel('l')
%     
%     figure
%     subplot(3,1,1)
%     plot(solution.time, solution.psid,'r');hold on; grid on;
%     plot(solution_constr.time, solution_constr.psid,'ob');
%     ylabel('thetad')
% 
%     subplot(3,1,2)
%     plot(solution.time, solution.l1d,'r');hold on; grid on;
%     plot(solution_constr.time, solution_constr.l1d,'ob');
%     ylabel('phid')
%     
%     subplot(3,1,3)
%     plot(solution.time, solution.l2d,'r'); hold on; grid on;
%     plot(solution_constr.time, solution_constr.l2d,'ob');
%     ylabel('ld')
        
    
    figure
    subplot(3,1,1)
    plot(solution.time, solution.p(1,:),'r') ; hold on;   grid on; 
    plot(solution_constr.time, solution_constr.p(1,:),'ob') ; hold on;    
    ylabel('X')
    
    subplot(3,1,2)
    plot(solution.time, solution.p(2,:),'r') ; hold on;  grid on;  
    plot(solution_constr.time, solution_constr.p(2,:),'ob') ; hold on;    
    ylabel('Y')
    
    subplot(3,1,3)
    plot(solution.time, solution.p(3,:),'r') ; hold on; grid on;   
    plot(solution_constr.time, solution_constr.p(3,:),'ob') ; hold on;
    ylabel('Z')
       
    
end

disp('Fleg')
solution.Fleg
opt_Tf
disp('target')
solution.achieved_target


save('optimOK.mat','solution','T_th','mu','N_dyn','Fleg_max', 'Fr_max', 'p0','pf', 'opt_Tf');


