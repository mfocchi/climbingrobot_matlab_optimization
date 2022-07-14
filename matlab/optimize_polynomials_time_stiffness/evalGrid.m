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
        [number_of_converged_solutions,  initial_kin_energy,  final_kin_energy, intEkin, opt_Fun, opt_Fut, opt_K, opt_Tf, T_pend,  solve_time] = eval_jump(pf, Fun_max, Fr_max, mu);
         

%         lthetaf_vector = [lthetaf_vector [l_range(k) ; thetaf_range(j)]];
%         feasible = [feasible number_of_feasible_solutions];    
%         converged = [converged number_of_converged_solutions];
%         initial_kin_energy = [initial_kin_energy opt_kin_energy] ;
%         final_kin_energy = [final_kin_energy opt_wasted] ;
     
          fprintf('y =%7.2f  | z =%7.2f |  conv=%2d |   Ekin0=%7.2f |  Ekinf = %7.2f  | intEkin = %7.2f |  Fun=%5.2f  | Fut=%5.2f | K=%5.2f | Tf=%5.2f | comp_time=%5.2f\n',...
                 [y_range(i); z_range(j); number_of_converged_solutions; initial_kin_energy;  final_kin_energy ;intEkin; opt_Fun; opt_Fut; opt_K;  opt_Tf; solve_time]);

    end
end
    
% l_range_dense = [l_range(1):0.01:l_range(end)];
% thetaf_range_dense = [thetaf_range(1):0.005:thetaf_range(end)];
% save('lookuptables.mat', 'lthetaf_vector', 'initial_kin_energy', 'final_kin_energy', 'feasible', 'converged', 'l_range_dense', 'thetaf_range_dense');
  