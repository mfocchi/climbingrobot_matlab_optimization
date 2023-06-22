
function [problem_solved, solution] = optimize_cpp(p0,  pf, Fleg_max, Fr_max, mu, jump_clearance) 


    global m  g w1 w2 w3 w4 w5 w6 num_params FRICTION_CONE b p_a1 p_a2 T_th N_dyn int_method int_steps contact_normal obstacle_avoidance
    
    
    obstacle_avoidance = false;

    %WORLD FRAME ATTACHED TO ANCHOR 1
    anchor_distance = 5;
    b = anchor_distance;
    p_a1 = [0;0;0];
    p_a2 = [0;anchor_distance;0];
    g = 9.81;
    m = 5.08;   % Mass [kg]
    T_th = 0.05;
    contact_normal =[1;0;0];

    %make it column vector
    p0 = p0(:);
    pf = pf(:);
    dt = 0.001; %for eval solution

    %optim parameters
    w1 = 1 ; % green initial cost (not used)
    w2 = 1; %red final cost (not used)
    w3 = 1 ; % slacks energy weight E
    w4 = 1; % diff Fr1/2 smothing
    w5 = 1; %ekinf (important! energy has much higher values!)
    w6 = 1; %Fr work
    constr_tolerance = 1e-3;
    %accurate
    int_method = 'rk4';
    int_steps = 5; %0 means normal intergation
    N_dyn = 30; %dynamic constraints (discretization) 
    FRICTION_CONE = 1;


    
    %compute initial state from jump param
    x0 = computeStateFromCartesian(p0);

    %pendulum period
    T_pend = 2*pi*sqrt(x0(2)/g)/4; % half period TODO replace with linearized x0(2) = l10
    num_params = 4;    
    Fr_l0 = 0*ones(1,N_dyn);
    Fr_r0 = 0*ones(1,N_dyn);
    x0 = [  Fleg_max,  Fleg_max,  Fleg_max,        T_pend,  Fr_l0,                               Fr_r0]; %opt vars=   Flegx Flegy Flexz Tf  traj_Fr_l traj_Fr_r
    lb = [  -Fleg_max,   -Fleg_max, -Fleg_max          0.01, -Fr_max*ones(1,N_dyn), -Fr_max*ones(1,N_dyn)];
    ub = [  Fleg_max,    Fleg_max, Fleg_max,           inf,  0*ones(1,N_dyn),            0*ones(1,N_dyn)];

    options = optimoptions('fmincon','Display','iter','Algorithm','sqp',  ... % does not always satisfy bounds
    'MaxFunctionEvaluations', 10000, 'ConstraintTolerance', constr_tolerance);

    tic
    [x, final_cost, EXITFLAG, output] = fmincon(@(x) cost(x, p0,  pf), x0,[],[],[],[],lb,ub,  @(x) constraints(x, p0,  pf, Fleg_max, Fr_max, mu, jump_clearance) , options);
    toc

    solution = eval_solution(x, dt,  p0, pf) ;
    solution.T_th = T_th;
    problem_solved = (EXITFLAG == 1) || (EXITFLAG ==2) ;
    
    %save('test_matlab2.mat','solution','mu','Fleg_max', 'Fr_max', 'p0','pf');



end

