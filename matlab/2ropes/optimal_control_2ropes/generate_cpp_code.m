clear all; close all

%cd to actual dir
filePath = matlab.desktop.editor.getActiveFilename;
pathparts = strsplit(filePath,filesep);
dirpath= pathparts(1:end-1);
actual_dir =  strjoin(dirpath,"/");
cd(actual_dir);

% normal test
mass = 5.08; 
Fleg_max = 300;
Fr_max = 90; % Fr is negative

% %landing test
% mass = 15.0246; 
% Fleg_max =  600;
% Fr_max = 300; % Fr is negative



mu = 0.8;

params.jump_clearance = 1;
params.m = mass;   % Mass [kg]
params.obstacle_avoidance  = false;
anchor_distance = 5;
params.num_params = 4.;   

%accurate
params.int_method = 'rk4';
params.N_dyn = 30.; %dynamic constraints (number of knowts in the discretization) 
params.FRICTION_CONE = 1;
params.int_steps = 5.; %0 means normal intergation
params.contact_normal =[1;0;0];
params.b = anchor_distance;
params.p_a1 = [0;0;0];
params.p_a2 = [0;anchor_distance;0];
params.g = 9.81;
params.w1 =1; % green initial cost (not used)
params.w2=1;%red final cost (not used)
params.w3=1;
params.w4=1;% diff Fr1/2 smothing
params.w5=1; %ekinf (important! energy has much higher values!)
params.w6=1;%Fr work
params.T_th =  0.05;

%jump params
% INITIAL POINT
p0 = [0.5, 2.5, -6]; % there is singularity for px = 0!

%FINAL TARGET
pf= [0.5, 4,-4];

% it should give the same result as optimal control 2 ropes (for sanity
% check)
%solution.Tf =1.2175
%solution.achieved_target(normal test) =0.5197  3.9967 -4.0008

%[problem_solved, solution] = optimize_cpp(p0,  pf, Fleg_max, Fr_max, mu, params) 


% generates the cpp code
% run the mex generator after calling optimize_cpp otherwise he complains it is missing the pa1 
cfg = coder.config('mex');
cfg.IntegrityChecks = false;
cfg.SaturateOnIntegerOverflow = false;
%codegen -config cfg  optimize_cpp -args {[0, 0, 0], [0, 0, 0], 0, 0, 0, coder.cstructname(params, 'param') } -nargout 1 -report

%it gives a slightly different result than optimal_control_2ropes:
%solution.Tf = 1.3234
%solution.achieved_target(normal test) = 0.5971     3.9923  -4.0035
solution = optimize_cpp_mex(p0,  pf, Fleg_max, Fr_max, mu, params);
solution.Tf
solution.achieved_target

subplot(2,1,1)           
plot(solution.time, solution.Fr_l, 'ko-'); grid on;hold on;  ylabel('deltaFrl'); grid on;hold on;

subplot(2,1,2)   
plot(solution.time, solution.Fr_r, 'bo-'); grid on;hold on;  ylabel('deltaFrr'); grid on;hold on;


%save('../simulation/compact_model/tests/test_matlab2_cpp.mat','solution','mu','Fleg_max', 'Fr_max', 'p0','pf');
%system('python3 test_mex.py');

copyfile codegen ~/trento_lab_home/ros_ws/src/trento_lab_framework/locosim/robot_control/base_controllers/
copyfile optimize_cpp_mex.mexa64 ~/trento_lab_home/ros_ws/src/trento_lab_framework/locosim/robot_control/base_controllers/codegen/