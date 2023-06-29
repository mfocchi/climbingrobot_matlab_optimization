
function [x, EXITFLAG, final_cost] = optimize_cpp_mpc(actual_state, actual_t, ref_com, Fr_l0, Fr_r0, Fr_max, mpc_N)  
    
        global m  g w1 w2  b p_a1 p_a2  mpc_dt int_method int_steps contact_normal 
        constr_tolerance = 1e-3;

          delta_Fr_l0 = 0*ones(1,mpc_N);
        delta_Fr_r0 = 0*ones(1,mpc_N);
        x0 = [   delta_Fr_l0,                    delta_Fr_r0]; %opt vars=   Flegx Flegy Flexz Tf  traj_Fr_l traj_Fr_r
        lb = [  -Fr_max*ones(1,mpc_N), -Fr_max*ones(1,mpc_N)];
        ub = [   Fr_max*ones(1,mpc_N),  Fr_max*ones(1,mpc_N)];
        options = optimoptions('fmincon','Display','none','Algorithm','sqp',  ... % does not always satisfy bounds
        'MaxFunctionEvaluations', 5000, 'ConstraintTolerance', constr_tolerance);

        %optim (comment this for sanity check test) 
        [x, final_cost, EXITFLAG, output] = fmincon(@(x) cost_mpc(x, actual_state, actual_t, ref_com, Fr_l0, Fr_r0, mpc_N),  x0,[],[],[],[],lb,ub, [], options);%,  @(x) constraints_mpc(x, actual_com, ref_com, Fr_l0, Fr_r0 ) , options);

end

