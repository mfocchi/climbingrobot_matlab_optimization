


%%

clear all; close all

%cd to actual dir
filePath = matlab.desktop.editor.getActiveFilename;
pathparts = strsplit(filePath,filesep);
dirpath= pathparts(1:end-1);
actual_dir =  strjoin(dirpath,"/");
cd(actual_dir);

% physical limits
Fleg_max = 300;
Fr_max = 90; % Fr is negative
mu = 0.8;
T_th = 0.05;
contact_normal =[1;0;0];
jump_clearance = 1;

%jump params
% INITIAL POINT
p0 = [0.5, 2.5, -6]; % there is singularity for px = 0!

%FINAL TARGET
pf= [0.5, 4,-4];

% it should give the same result as optimal control 2 ropes (for sanity
% check)
%solution.Tf =1.2175
%solution.achieved_target(normal test) =0.5197  3.9967 -4.0008
[problem_solved, solution] = optimize_cpp(p0,  pf, Fleg_max, Fr_max, mu, jump_clearance) 


% generates the cpp code
% run the mex generator after calling optimize_cpp otherwise he complains it is missing the pa1 
cfg = coder.config('mex');
cfg.IntegrityChecks = false;
cfg.SaturateOnIntegerOverflow = false;
codegen -config cfg  optimize_cpp -args {[0, 0, 0], [0, 0, 0], 0, 0, 0,0 } -nargout 2 -report

%it gives a slightly different result than optimal_control_2ropes:
%solution.Tf = 1.3234
%solution.achieved_target(normal test) = 0.5971     3.9923  -4.0035
[problem_solved, solution] = optimize_cpp_mex(p0,  pf, Fleg_max, Fr_max, mu, jump_clearance) 
save('../simulation/compact_model/tests/test_matlab2_cpp.mat','solution','mu','Fleg_max', 'Fr_max', 'p0','pf');
