%
clear all ; close all ; clc

% physical limits
Fun_max =15;
Fr_max =180; % Fr in negative
mu = 0.8;

%jump paramaters
anchor_pos =[ 0.149937507812035;
          0;
     -2.9962507811849];
jump_lenght_z = -17;
jump_lenght_y = 10;
N = 4; % grid  discretization
  
y_range = linspace(anchor_pos(2), anchor_pos(2)+jump_lenght_y, N);
z_range = linspace(anchor_pos(3), anchor_pos(3)+jump_lenght_z, N);
%fixed target
pf = [0.001; y_range(end); z_range(end)];

max_n_jumps = 4;
n_states = N^2; % combinations

value_function = inf(max_n_jumps, n_states);
value_function_N = [];
x = [];

% the initial cost fot the target is zero, and the value function is not
% infinite because I can get to the target from all the states, so we skip
% the initial step

% compute state space 
for i = 1:length(y_range)    
    for j = 1:length(z_range)  
        %cycle on inital point
        p0 = [anchor_pos(1); y_range(i); z_range(j)];
        x = [x, p0];
    end   
end

% compute value function at final state (t = njumps) 
% we suppose we will arrive at the target at that stage
value_function(max_n_jumps, end) = 0;

% iterate for all the other 
for k = max_n_jumps-1:-1:1
    % for each state compute the value functions 
    for i=1:n_states        
        for j=1:n_states
            p0 = x(:,i);
            pf = x(:,j);
            [number_of_converged_solutions,  initial_kin_energy,  final_kin_energy, intEkin, opt_Fun, opt_Fut, Fr, opt_K, opt_Tf, T_pend,  solve_time] = eval_jump(p0, pf, Fun_max, Fr_max, mu);
            cost = eval_jump_cost([1,1,10,10], intEkin, final_kin_energy, Fun_max, opt_Fun, Fr_max, Fr, number_of_converged_solutions);
            q_cost(j) = cost + value_function(k+1, j);
        end
        value_function(k, i) = min(q_cost);
        
    end
end

%save('value_function.mat','value_function','x','max_n_jumps','Fun_max','Fr_max','mu','n_states')



load('value_function.mat')

%to get the optimal set of jumps we minimize from the initial state and the
%initial time_step the sequence action transition is given by the min value
%function 
% at time =1 we start in state 1 (anchor)
index(1) = 1;
for k = 1:max_n_jumps-1
       
       q_cost = [];
        for j=1:n_states
            
            p0 = x(:,index(k));
            pf = x(:,j);
            [number_of_converged_solutions,  initial_kin_energy,  final_kin_energy, intEkin, opt_Fun, opt_Fut, Fr, opt_K, opt_Tf, T_pend,  solve_time] = eval_jump(p0, pf, Fun_max, Fr_max, mu);
            cost = eval_jump_cost([1,1,10,10], intEkin, final_kin_energy, Fun_max, opt_Fun, Fr_max, Fr, number_of_converged_solutions);
            q_cost(j) = cost + value_function(k+1, j);
            
        end
        [m,index(k+1)] = min(q_cost);
        
end    
    
optimal_traj = x(:,index)    
    

plot(optimal_traj(2,:),optimal_traj(3,:),'Marker', '.', 'Color','b', 'MarkerSize',60) ;   grid on; hold on;
plot( anchor_pos(2), anchor_pos(3), 'Marker', '.', 'Color','g', 'MarkerSize',60) ;
plot( pf(2), pf(3), 'Marker', '.', 'Color','r', 'MarkerSize',60) ;
    
    
    
 