%
clear all ; close all ; clc

% physical limits
Fun_max =15;
Fr_max =50; % Fr in negative
mu = 0.8;

%jump paramaters
anchor_pos =[ 0.149937507812035; 0;-2.9962507811849];
target_pos = [0; 10;-20];

% generate random rargets
N = 100;
R = 6;
t = 2*pi*rand(N,1);
r = R*sqrt(rand(N,1));
y_range = target_pos(2) + r.*cos(t);
z_range = target_pos(3) + r.*sin(t);
scatter(y_range,z_range, 1000, 'b','.') ;
xlabel('Y')
ylabel('Z');hold on;grid on;
plot(target_pos(2),target_pos(3), 'Marker', '.', 'Color','r', 'MarkerSize',40) ;


for i = 1:length(y_range)    
    for j = 1:length(z_range)  
        %cycle on final point
        pf = [0.001; y_range(i); z_range(j)];
     
        [number_of_converged_solutions,  initial_kin_energy,  final_kin_energy, intEkin, opt_Fun, opt_Fut, Fr, opt_K, opt_Tf, T_pend,  solve_time, efficiency] = eval_jump(anchor_pos, pf, Fun_max, Fr_max, mu);
        total_cost = eval_jump_cost([1,1,10,10], intEkin, final_kin_energy, Fun_max, opt_Fun, Fr_max, Fr, number_of_converged_solutions);
    
        fprintf('total_cost=%7.2f | y =%7.2f  | z =%7.2f |  conv=%4d |   Ekin0=%7.2f |  Ekinf = %7.2f  | intEkin = %7.2f |  Fun=%5.2f  | Fut=%5.2f | Fr=%7.2f | K=%5.2f | Tf=%5.2f | comp_time=%5.2f \n',...
                 [total_cost; y_range(i); z_range(j); number_of_converged_solutions; initial_kin_energy;  final_kin_energy ;intEkin; opt_Fun; opt_Fut;  Fr; opt_K;  opt_Tf; solve_time]);
        
    end   
end
 