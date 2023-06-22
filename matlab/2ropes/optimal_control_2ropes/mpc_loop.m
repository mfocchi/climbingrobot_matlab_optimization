clc; clear all;

load test_matlab2.mat
global m  g w1 w2 w3 w4 w5 w6  b p_a1 p_a2  mpc_dt int_method int_steps contact_normal 



Fleg_max = 300;
Fr_max = 20; % Fr is negative (max variation)
contact_normal =[1;0;0];
jump_clearance = 1;

%accurate
int_method = 'rk4';
int_steps = 5; %0 means normal intergation
constr_tolerance = 1e-3;
dt=0.001; % only to evaluate solution

%WORLD FRAME ATTACHED TO ANCHOR 1
anchor_distance = 5;
b = anchor_distance;
p_a1 = [0;0;0];
p_a2 = [0;anchor_distance;0];
g = 9.81;
m = 5.08;   % Mass [kg]

N_dyn = length(solution.time);
mpc_N = 0.4*length(solution.time);
mpc_dt = solution.Tf / (N_dyn-1);

samples = length(solution.time) - mpc_N;
% 0.3*rand(3,1); 

actual_t = 0;
actual_state = [solution.psi(:,3),solution.l1(:,3),solution.l2(:,3), solution.psid(:,3), solution.l1d(:,3), solution.l2d(:,3)];

% 1 - sanity check: on traj consider after the application of T_th!
% (sample > 2)
% [cost, pos] = cost_mpc(zeros(1,2*mpc_N), actual_state,  actual_t, solution.p(3:end), solution.Fr_l(3:end), solution.Fr_r(3:end),  mpc_N);
% pos
% solution.p(:,3:samples)



for i=3:samples
    ref_com = solution.p(:,i:i+mpc_N-1);      
    Fr_l0 = solution.Fr_l(:,i:i+mpc_N-1);
    Fr_r0 = solution.Fr_r(:,i:i+mpc_N-1);
    
  
    delta_Fr_l0 = 0*ones(1,mpc_N);
    delta_Fr_r0 = 0*ones(1,mpc_N);
    x0 = [   delta_Fr_l0,                    delta_Fr_r0]; %opt vars=   Flegx Flegy Flexz Tf  traj_Fr_l traj_Fr_r
    lb = [  -Fr_max*ones(1,mpc_N), -Fr_max*ones(1,mpc_N)];
    ub = [   Fr_max*ones(1,mpc_N),  Fr_max*ones(1,mpc_N)];
    options = optimoptions('fmincon','Display','iter','Algorithm','sqp',  ... % does not always satisfy bounds
    'MaxFunctionEvaluations', 10000, 'ConstraintTolerance', constr_tolerance);

     %cost_mpc(x0, actual_state, actual_t,ref_com, Fr_l0, Fr_r0, mpc_N);

%     tic
%     [x, final_cost, EXITFLAG, output] = fmincon(@(x) cost_mpc(x, actual_com, actual_t, ref_com, Fr_l0, Fr_r0), x0,[],[],[],[],lb,ub, options);%,  @(x) constraints_mpc(x, actual_com, ref_com, Fr_l0, Fr_r0 ) , options);
%     toc
    
     actual_t = actual_t + mpc_dt; 
      
     
     
    % sanity check : uncomment one of the two
     %i
     %ref_com(:,1) % need to consider the next sample
     %actual_com = computePositionVelocity(actual_state(1), actual_state(2), actual_state(3))
     
     % 2 - sanity check with  for loop and integrate dynamics
     %[actual_state, actual_t] = integrate_dynamics(actual_state ,actual_t, mpc_dt/(int_steps-1), int_steps, (Fr_l0(1) + delta_Fr_l0(1))*ones(1,int_steps),  (Fr_r0(1) + delta_Fr_r0(1))*ones(1,int_steps),[0;0;0], int_method); % keep Fr constant   
     % 3 - sanity check with for loop and computeRollout
      [state12, time12] = computeRollout(actual_state, actual_t,mpc_dt, 2, Fr_l0(1) + delta_Fr_l0(1), Fr_r0(1) + delta_Fr_r0(1),[0;0;0],int_method,int_steps);  
      actual_state = state12(:,2); 

     

end

