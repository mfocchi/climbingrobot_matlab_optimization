cfg = coder.config('mex');
cfg.IntegrityChecks = false;
cfg.SaturateOnIntegerOverflow = false;


codegen -config cfg  optimize_cpp -args {0,0,0,[0, 0, 0],0,0,0} -nargout 7 -report


% physical limits
Fun_max =150;
Fr_max =80; % Fr in negative
mu = 0.8;
% Marco Frego test: initial state
l_0 = 3;
theta0 =atan2(0.38, l_0);
%theta0 = 0.05; 
phi0 = 0 ;

% Marco Frego test: final state
pf = [0.001 5 -8];
[number_of_converged_solutions,  initial_kin_energy,  final_kin_energy,  opt_Fun, opt_Fut, opt_K, opt_Tf] = optimize_cpp_mex(l_0, theta0, phi0,  pf, Fun_max, Fr_max, mu) 