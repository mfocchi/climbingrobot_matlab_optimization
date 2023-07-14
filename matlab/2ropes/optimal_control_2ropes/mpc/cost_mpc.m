function cost = cost_mpc(x, state0,  actual_t, ref_com, Fr_l0, Fr_r0,mpc_N, params)

    % init for cpp
    ref_com_mpc = zeros(3,mpc_N);
    cost=0;
    
  
    
    if (length(ref_com) < mpc_N) 
        disp('cost_mpc:wrong ref_com input length: ref_com should be longer than mpc_N')
        ref_com
        return
    end
    
    delta_Fr_l = x(1:mpc_N); 
    delta_Fr_r = x(mpc_N+1:2*mpc_N); 
    
    ref_com_mpc = ref_com(:, 1:mpc_N);
       
       
    [p, pd, t] = eval_pos_vel_mpc(state0,  actual_t,  Fr_l0, Fr_r0,delta_Fr_l, delta_Fr_r, mpc_N, params);
    
    
    %p has mpc_N +1 elements 
    % track
    tracking = sum (vecnorm(ref_com_mpc - p).^2);
    
    % smoothnes: minimize jerky control action
    smooth = sum(delta_Fr_l.^2) + sum(delta_Fr_r.^2);
         
    cost =  params.w1* tracking;% + w2 *smooth ;
end