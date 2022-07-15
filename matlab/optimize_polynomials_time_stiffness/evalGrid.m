%
clear all ; close all ; clc

% physical limits
Fun_max =15;
Fr_max =180; % Fr in negative
mu = 0.8;

y_range = [0:2:10];
z_range = [-20:2:-4];

% converged = [];
% initial_kin_energy = [];
% final_kin_energy = [];

for i = 1:length(y_range)    
    for j = 1:length(z_range)    
        pf = [0.001; y_range(i); z_range(j)];
        [number_of_converged_solutions,  initial_kin_energy,  final_kin_energy, intEkin, opt_Fun, opt_Fut, Fr, opt_K, opt_Tf, T_pend,  solve_time] = eval_jump(pf, Fun_max, Fr_max, mu);
        
        total_cost = eval_jump_cost([1,1,10,10], intEkin, final_kin_energy, Fun_max, opt_Fun, Fr_max, Fr, number_of_converged_solutions);
    
        fprintf('total_cost=%7.2f | y =%7.2f  | z =%7.2f |  conv=%4d |   Ekin0=%7.2f |  Ekinf = %7.2f  | intEkin = %7.2f |  Fun=%5.2f  | Fut=%5.2f | Fr=%7.2f | K=%5.2f | Tf=%5.2f | comp_time=%5.2f \n',...
                 [total_cost; y_range(i); z_range(j); number_of_converged_solutions; initial_kin_energy;  final_kin_energy ;intEkin; opt_Fun; opt_Fut;  Fr; opt_K;  opt_Tf; solve_time]);

        
    end   
    
end