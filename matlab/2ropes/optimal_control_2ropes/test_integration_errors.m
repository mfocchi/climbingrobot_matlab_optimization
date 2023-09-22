clear all; close all; clc

%cd to actual dir
filePath = matlab.desktop.editor.getActiveFilename;
pathparts = strsplit(filePath,filesep);
dirpath= pathparts(1:end-1);
actual_dir =  strjoin(dirpath,"/");
cd(actual_dir);

%jump params
p0 = [0.5, 2.5, -6]; % there is singularity for px = 0!
pf= [0.5, 4,-4];


tests = {'exp1',40, 'rk4'  ,  0;
         'exp2',60, 'rk4'  ,  0; 
        'exp3', 40, 'rk4'  ,  5;
        'exp4', 40, 'eul',  0;
        'exp4', 40, 'eul',  5;
        'exp5', 40, 'eul',  10;
        'exp6', 30, 'rk4'  , 5};
  
        
          
for n_test =1:size(tests,1)

    params.jump_clearance = 1;
    mass = 5.08; 
    Fleg_max = 300;
    Fr_max = 90; % Fr is negative
    mu = 0.8;
    params.m = mass;   % Mass [kg]
    params.obstacle_avoidance  = false;
    params.obstacle_location = [-0.5; 3;-7.5];
    anchor_distance = 5;
    params.num_params = 4.;   
    params.int_method = tests{n_test, 3};
    params.N_dyn = tests{n_test, 2}; %dynamic constraints (number of knowts in the discretization) 
    params.FRICTION_CONE = 1;
    params.int_steps = tests{n_test, 4}; %0 means normal intergation
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

    %it gives a slightly different result than optimal_control_2ropes:
    %solution.Tf = 1.3234
    %solution.achieved_target(normal test) = 0.5971     3.9923  -4.0035
    solution = optimize_cpp_mex(p0,  pf, Fleg_max, Fr_max, mu, params);
    % solution.Tf
    % solution.achieved_target
    plot_solution(solution, p0, pf, Fleg_max, Fr_max, mu, params) 

    % 
    % 1 First-order optimality measure was less than options.OptimalityTolerance, and maximum constraint violation was less than options.ConstraintTolerance.
    % 0 Number of iterations exceeded options.MaxIterations or number of function evaluations exceeded options.MaxFunctionEvaluations.
    % -1 Stopped by an output function or plot function.
    % -2 No feasible point was found.
    % 2 Change in x was less than options.StepTolerance and maximum constraint violation was less than options.ConstraintTolerance.

    fprintf(2,"%s,   N: %d, method : %s,  int_steps: %d \n", tests{n_test,1}, tests{n_test,2},tests{n_test,3}, tests{n_test,4} );
    
    switch solution.problem_solved
        case 1 
            fprintf(1,"Problem converged!\n")
        case -2  
            fprintf(2,"Problem didnt converge!\n")
        case 2 
            fprintf(1,"semidefinite solution (should modify the cost)\n")
        case 0 
            fprintf(2,"Max number of feval exceeded (10000)\n")
    end

    fprintf(1,"number of iterations: %i\n", solution.optim_output.iterations);
    %fprintf(1,"number of func evaluations: %i\n", solution.optim_output.funcCount);
    fprintf('max_integration_error:  %f\n', norm(solution.p(:,end) - solution.solution_constr.p(:,end)));
    fprintf('max_error:  %f\n\n', norm(solution.p(:,end) - pf(:)));

end