


%%

clear all 

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
pf= [0.5, 4,-2];

%[problem_solved, solution] = optimize_cpp(p0,  pf, Fleg_max, Fr_max, mu, jump_clearance) 
%Tf = 1.1654
%solution.achieved_target = [0.5036, 3.9865, -4.0086] 


% run the mex generator after calling optimize_cpp otherwise he complains
% it is missing the pa1
  
% cfg = coder.config('mex');
% cfg.IntegrityChecks = false;
% cfg.SaturateOnIntegerOverflow = false;
% codegen -config cfg  optimize_cpp -args {[0, 0, 0], [0, 0, 0], 0, 0, 0,0 } -nargout 2 -report

[problem_solved, solution] = optimize_cpp_mex(p0,  pf, Fleg_max, Fr_max, mu, 2.5) 
save('test_matlab3.mat','solution','mu','Fleg_max', 'Fr_max', 'p0','pf');
