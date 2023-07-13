clc; clear all;

load('../simulation/compact_model/tests/test_matlab2.mat')
global m  g w1 w2  b p_a1 p_a2  mpc_dt int_method int_steps contact_normal 

DEBUG_DYNAMICS = false;
DEBUG_MPC_MACHINERY = false;

w1 =1;
w2=1;

Fr_max = 50; % Fr is negative (max variation)
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
max_feval = 5000;
samples = length(solution.time) - mpc_N+1;


start_mpc = 3;
actual_t = solution.time(start_mpc);
actual_state = [solution.psi(:,start_mpc);solution.l1(:,start_mpc);solution.l2(:,start_mpc); solution.psid(:,start_mpc); solution.l1d(:,start_mpc); solution.l2d(:,start_mpc)];


if DEBUG_DYNAMICS
   % 1 - sanity check: on traj consider after the application of T_th!
   % (sample > 2)
    pos = eval_pos_vel_mpc(actual_state,  actual_t, solution.Fr_l(start_mpc:end), solution.Fr_r(start_mpc:end),zeros(1,mpc_N),zeros(1,mpc_N), mpc_N)
    solution.p(:,3:samples)
end


for i=start_mpc:samples
    ref_com = solution.p(:,i:i+mpc_N-1);      
    Fr_l0 = solution.Fr_l(:,i:i+mpc_N-1);
    Fr_r0 = solution.Fr_r(:,i:i+mpc_N-1);
    fprintf("Iteration #: %d\n", i)
    
    if ~DEBUG_DYNAMICS
        
        %debug mpc machinery
        if DEBUG_MPC_MACHINERY
            delta_Fr_l = zeros(1,mpc_N);
            delta_Fr_r = zeros(1,mpc_N);
        else

            %ADD NOISE
             %compute position relative to actualstate      
             [act_p] = computePositionVelocity(actual_state(1), actual_state(2), actual_state(3));
            
             %adding noise only on position!
             %1 - random noise 
             %act_p =  act_p + [0.2*rand(1);0.2*rand(1); 0.2*rand(1)];
             % 2 deterninistic noise (only errors on Y and Z are corrected because of
             % underactuation!)
             act_p =  act_p + [0.;0.1; 0.0];

             %overwrite the position part of the state after adding noise
             act_state_pos_noise = computeStateFromCartesian(act_p);
             actual_state(1:3) = act_state_pos_noise(1:3);
             %%%%%%%%%%%%%%%%%%%%%%%

             %Optimization
             [x, EXITFLAG, final_cost] = optimize_cpp_mpc(actual_state, actual_t, ref_com, Fr_l0, Fr_r0, Fr_max, mpc_N);

             delta_Fr_l = x(1:mpc_N);
             delta_Fr_r = x(mpc_N+1:2*mpc_N);
        end

        % predict new traj
        [mpc_p, mpc_pd, mpc_time] = eval_pos_vel_mpc(actual_state,  actual_t, Fr_l0, Fr_r0,delta_Fr_l ,delta_Fr_r, mpc_N);

        %update dynamics (this emulates the real dynamics with noise)
        [actual_state, actual_t] = integrate_dynamics(actual_state ,actual_t, mpc_dt/(int_steps-1), int_steps, (Fr_l0(1) + delta_Fr_l(1))*ones(1,int_steps),  (Fr_r0(1) + delta_Fr_r(1))*ones(1,int_steps),[0;0;0], int_method); 


        %actual_com = computePositionVelocity(actual_state(1), actual_state(2), actual_state(3));   

        %plot
        clf(gcf)
        set(0, 'DefaultAxesBox', 'on');
        set(0, 'DefaultTextFontSize', 30);
        set(0, 'DefaultAxesFontSize', 30);
        set(0, 'DefaultUicontrolFontSize', 30);    
        %ref signal
        subplot(3,1,1)
        ylabel('X')
        plot(solution.time(start_mpc:end), solution.p(1,start_mpc:end), 'ro-'); grid on;hold on;
        plot(mpc_time, mpc_p(1,:), 'bo-'); grid on;hold on;

        subplot(3,1,2)
        ylabel('Y')
        plot(solution.time(start_mpc:end), solution.p(2,start_mpc:end), 'ro-'); grid on;hold on;
        plot(mpc_time, mpc_p(2,:), 'bo-'); grid on;hold on;

        subplot(3,1,3)
        ylabel('Z')
        plot(solution.time(start_mpc:end), solution.p(3,start_mpc:end), 'ro-'); grid on;hold on;
        plot(mpc_time, mpc_p(3,:), 'bo-'); grid on;hold on;

        pause(0.3)

    else

        % sanity check : uncomment one of the two
         ref_com(:,1) % need to consider the next sample
         actual_com = computePositionVelocity(actual_state(1), actual_state(2), actual_state(3))

         % 2 - sanity check with  for loop and integrate dynamics
         [actual_state, actual_t] = integrate_dynamics(actual_state ,actual_t, mpc_dt/(int_steps-1), int_steps, (Fr_l0(1) )*ones(1,int_steps),  (Fr_r0(1) )*ones(1,int_steps),[0;0;0], int_method); % keep Fr constant   
         % 3 - sanity check with for loop and computeRollout
         % [state12, time12] = computeRollout(actual_state, actual_t,mpc_dt, 2, Fr_l0(1) , Fr_r0(1)  ,[0;0;0],int_method,int_steps);  
         % actual_state = state12(:,2);   
    end
     
end

