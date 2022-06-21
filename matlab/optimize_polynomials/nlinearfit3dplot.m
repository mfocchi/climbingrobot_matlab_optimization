


%
clear all ; close all ; clc
global m  g  w1 w2 w3  N  OLD_FORMULATION POLY_TYPE num_params
m = 5;
g = 9.81;

% physical limits
Fun_max = 30 ;
mu = 0.5;
tol = .1;

w1 = 1 ; % green initial
w2 = 0.6; %red final
w3 = 0.01 ;
N = 10 ; % energy constraints

dt=0.001;
OLD_FORMULATION = 1;
POLY_TYPE = 0; % 0 cubic, 1 quintic

if (POLY_TYPE)
    num_params = 12;
else 
    num_params = 8; 
end



l_range = [2:1:10];
thetaf_range = [0.1:0.1:0.9];

lthetaf_vector = [];
feasible =[];
converged = [];
initial_kin_energy = [];
final_kin_energy = [];

for k=1:length(l_range)    
    for j=1:length(thetaf_range)        
        
        [number_of_feasible_solutions,number_of_converged_solutions,  opt_kin_energy,  opt_wasted, opt_Fun, opt_Fut] = eval_jump(l_range(k), thetaf_range(j), dt, tol, Fun_max, mu);
              
        lthetaf_vector = [lthetaf_vector [l_range(k) ; thetaf_range(j)]];
        feasible = [feasible number_of_feasible_solutions];    
        converged = [converged number_of_converged_solutions];
        initial_kin_energy = [initial_kin_energy opt_kin_energy] ;
        final_kin_energy = [final_kin_energy opt_wasted] ;
     
        
        fprintf('l =%3.2f    thetaf =%5.2f    feas=%5d    conv=%5d    Ekin0=%5.2f   Ekinf = %5.2f    Fun=%5.2f   Fut=%5.2f \n',...
                 l_range(k), thetaf_range(j), number_of_feasible_solutions, number_of_converged_solutions, opt_kin_energy,  opt_wasted, opt_Fun, opt_Fut);

    end
end
    
l_range_dense = [l_range(1):0.01:l_range(end)];
thetaf_range_dense = [thetaf_range(1):0.005:thetaf_range(end)];

plot_surf("feasible", l_range_dense, thetaf_range_dense, lthetaf_vector(1,:) ,   lthetaf_vector(2,:),    feasible);
plot_surf("conv",     l_range_dense, thetaf_range_dense, lthetaf_vector(1,:) ,   lthetaf_vector(2,:),    converged);
plot_surf("initial kin energy",l_range_dense, thetaf_range_dense, lthetaf_vector(1,:) ,   lthetaf_vector(2,:),    initial_kin_energy);
plot_surf("final kin energy",l_range_dense, thetaf_range_dense, lthetaf_vector(1,:) ,   lthetaf_vector(2,:),    final_kin_energy);



save('lookuptables.mat', 'lthetaf_vector', 'initial_kin_energy', 'final_kin_energy', 'feasible', 'converged', 'l_range_dense', 'thetaf_range_dense');
  