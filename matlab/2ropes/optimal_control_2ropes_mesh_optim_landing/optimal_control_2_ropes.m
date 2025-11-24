clear all ; close all ; clc

%cd to actual dir if you are not already there
filePath = matlab.desktop.editor.getActiveFilename;
pathparts = strsplit(filePath,filesep);
dirpath= pathparts(1:end-1);
actual_dir =  strjoin(dirpath,"/");
cd(actual_dir);

USEGENCODE = true;
COPYTOLOCOSIM = true;
 

%Initial position
p0 = [0.5, 2.5, -6]; % there is singularity for px = 0!
%center of the selected landing patch
landing_patch_center = [0.5, 4,-4];
Fleg_max = 600;
Fr_max = 90; % max rope force 
Fr_min = 10; % min rope force 

% the order of params matters for code generation
params.m = 10.08;   % Mass [kg]
%WORLD FRAME ATTACHED TO ANCHOR 1 (left)
anchor_distance = 5;
params.num_params = 4;   
params.int_method = 'rk4';
params.N_dyn = 30; %dynamic constraints (number of knowts in the discretization) 
params.FRICTION_CONE = 1;
params.int_steps = 5.; %0 means normal intergation

params.b = anchor_distance;
params.p_a1 = [0;0;0];
params.p_a2 = [0;anchor_distance;0];
params.g = 9.81;
params.w1=1; % diff Fr1/2 smothing
params.w2=0; %hoist work
params.w3=300; % landing cost
params.T_th =  0.05;
params.obstacle_avoidance  =  'mesh'; %'none', 'mesh' %strings should have same length for code generation
params.jump_clearance = 1.0; % ensure at least this detachment from wall in the middle of the jump (not set for obstacle_avoidance = none) 

%generate mesh surface
wallDepth = 1; %how              
gridSize = 100;
maxRidgeDepth = 0.5;
seed= 47;
Lz = -20;         % Height of wall in meters
Ly = params.b;    % Width (horizontal extent) of wall in meters
[params.mesh_x , params.mesh_y, params.mesh_z] = generateRockWallMap(Lz, Ly, gridSize, wallDepth,maxRidgeDepth, seed, false);
[params.cost_x , params.cost_y, params.cost_z] = generateCostMap(Lz, Ly, gridSize, landing_patch_center + [0, -0.5, 0.5], 3);
params.patch_side =  1;


% Interpolator (note: z must be increasing — here from -10 to 0)
p0(1) = wallSurfaceEval(p0(3),p0(2),  params);
landing_patch_center(1) = wallSurfaceEval(landing_patch_center(3),landing_patch_center(2),  params);

% compute consistent normal 
params.contact_normal = wallNormalEval(p0(3),p0(2), params);

mu = 0.8;

%gen code (run if you did some change in the cost)
if USEGENCODE && (~isfile('optimize_cpp_mex.mexa64'))
    disp('Generating C++ code');
    % generates the cpp code
    % run the mex generator after calling optimize_cpp otherwise he complains it is missing the pa1 
    cfg = coder.config('mex');
    cfg.IntegrityChecks = false;
    cfg.SaturateOnIntegerOverflow = false;
    codegen -config cfg  optimize_cpp -args {[0, 0, 0], [0, 0, 0], 0, 0, 0, 0,  coder.cstructname(params, 'param') } -nargout 1 -report
    
end

mpc_fun   = 'optimize_cpp';
if USEGENCODE  
    mpc_fun=append(mpc_fun,'_mex' );
end
mpc_fun_handler = str2func(mpc_fun);
solution = mpc_fun_handler(p0,  landing_patch_center, Fleg_max, Fr_max, Fr_min,  mu, params);

switch solution.problem_solved
    case 1 
        fprintf(2,"Problem converged!\n")
    case -2  
        fprintf(2,"Problem didnt converge!\n")
   case 2 
        fprintf(2,"semidefinite solution (should modify the cost)\n")
    case 0 
        fprintf(2,"Max number of feval exceeded (10000)\n")
        
end
plot_curve( solution,solution.solution_constr, p0, landing_patch_center, mu,  false, 'r', true, params);

fprintf('Fleg:  %f %f %f \n\n',solution.Fleg(1), solution.Fleg(2), solution.Fleg(3))
fprintf('cost:  %f\n\n',solution.cost)
fprintf('final_kin_energy:  %f\n\n',solution.Ekinf)
fprintf('final_error_real:  %f\n\n',solution.final_error_real)

%-0.7472         0    0.7472   -0.8808         0    0.7849   -1.0812  
DEBUG = true;

if (DEBUG)
    eval_constraints(solution.c, solution.num_constr, solution.constr_tolerance, false)  
    figure
    
    plot(solution.time,0*ones(size(solution.Fr_l)),'k'); hold on; grid on;
    plot(solution.time,-Fr_max*ones(size(solution.Fr_l)),'k');
    plot(solution.time,solution.Fr_l,'r');
    plot(solution.time,solution.Fr_r,'b');
    legend({'min','max','Frl','Frr'});
    title('Forces')


    figure
    R = 0.025;
    Kt = 0.083;
    plot(solution.time,0*ones(size(solution.Fr_l)),'k'); hold on; grid on;
    plot(solution.time,-Fr_max*ones(size(solution.Fr_l))*R,'k');
    plot(solution.time,solution.Fr_l*R,'r');
    plot(solution.time,solution.Fr_r*R,'b');
    legend({'min','max','Left','Right'});
    title('Torque Nm')
   
    figure
    RADS2RPM = 60/(2*pi);
    Kt = 0.083;
    
    plot(solution.time,solution.l1d/R*RADS2RPM,'r'); hold on; grid on;
    plot(solution.time,solution.l2d/R*RADS2RPM,'b');
    legend({'Left','Right'});
 title('Speed RPM')
   

    figure
    subplot(3,1,1)
    plot(solution.time, solution.p(1,:),'r') ; hold on;   grid on; 
    plot(solution.solution_constr.time, solution.solution_constr.p(1,:),'ob') ; hold on;    
    ylabel('X')
    
    subplot(3,1,2)
    plot(solution.time, solution.p(2,:),'r') ; hold on;  grid on;  
    plot(solution.solution_constr.time, solution.solution_constr.p(2,:),'ob') ; hold on;    
    ylabel('Y')
    
    subplot(3,1,3)
    plot(solution.time, solution.p(3,:),'r') ; hold on; grid on;   
    plot(solution.solution_constr.time, solution.solution_constr.p(3,:),'ob') ; hold on;
    ylabel('Z')   
end

fprintf('Leg impulse force: %f %f %f\n\n', solution.Fleg);
fprintf('Jump Duration: %f\n\n', solution.Tf);
fprintf('Landing Target: %f %f %f\n\n', solution.achieved_target);
fprintf('Energy_consumed [J]: %f \n\n', solution.consumed_energy);
fprintf('Average Poiwer [W]: %f \n\n', solution.average_power);

%plot instantaneous power
% figure
% title("instantaneous_power")
% plot(solution.time_fine, solution.instantaneous_power);

if COPYTOLOCOSIM
    fprintf(2,"copying to locosim\n")
    copyfile codegen/mex/optimize_cpp/ ~/trento_lab_home/ros_ws/src/locosim/robot_control/base_controllers/climbingrobot_controller/codegen_mesh_landing/mex/optimize_cpp
    copyfile optimize_cpp_mex.mexa64 ~/trento_lab_home/ros_ws/src/locosim/robot_control/base_controllers/climbingrobot_controller/codegen_mesh_landing/
end

%1.384975
