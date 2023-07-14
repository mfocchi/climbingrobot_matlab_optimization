
function [problem_solved, solution] = optimize_cpp(p0,  pf, Fleg_max, Fr_max, mu, params) 
    %make it column vector
    p0 = p0(:);
    pf = pf(:);
    dt = 0.001; %for eval solution
    
    % needs to be fixed for code generation
    constr_tolerance = 1e-3;
    
    %compute initial state from jump param
    x0 = computeStateFromCartesian(params, p0);

    %pendulum period
    T_pend = 2*pi*sqrt(x0(2)/params.g)/4; % half period TODO replace with linearized x0(2) = l10
    params.num_params = 4;    
    Fr_l0 = 0*ones(1,params.N_dyn);
    Fr_r0 = 0*ones(1,params.N_dyn);
    x0 = [  Fleg_max,  Fleg_max,  Fleg_max,        T_pend,  Fr_l0,                               Fr_r0]; %opt vars=   Flegx Flegy Flexz Tf  traj_Fr_l traj_Fr_r
    lb = [  -Fleg_max,   -Fleg_max, -Fleg_max          0.01, -Fr_max*ones(1,params.N_dyn), -Fr_max*ones(1,params.N_dyn)];
    ub = [  Fleg_max,    Fleg_max, Fleg_max,           inf,  0*ones(1,params.N_dyn),            0*ones(1,params.N_dyn)];

    options = optimoptions('fmincon','Display','iter','Algorithm','sqp',  ... % does not always satisfy bounds
    'MaxFunctionEvaluations', 10000, 'ConstraintTolerance', constr_tolerance);

    tic
    [x, final_cost, EXITFLAG, output] = fmincon(@(x) cost(x, p0,  pf, params), x0,[],[],[],[],lb,ub,  @(x) constraints(x, p0,  pf, Fleg_max, Fr_max, mu, params) , options);
    toc

    solution = eval_solution(x, dt,  p0, pf, params) ;
    solution.T_th = params.T_th;
    problem_solved = (EXITFLAG == 1) || (EXITFLAG ==2) ;
    
    %save('test_matlab2.mat','solution','mu','Fleg_max', 'Fr_max', 'p0','pf');



end

