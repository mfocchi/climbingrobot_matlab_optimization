function [cost,p] = cost_mpc(x, state0,  actual_t, ref_com, Fr_l0, Fr_r0,mpc_N)

    global w1 w2  mpc_dt int_method  int_steps 

    if (length(ref_com) < mpc_N) || (length(Fr_l0) < mpc_N) || (length(Fr_r0) < mpc_N)
        disp('wrong input length: input should be longer than mpc_N')
        return
    end
      
    delta_Fr_l = x(1:mpc_N); 
    delta_Fr_r = x(mpc_N+1:2*mpc_N); 

    % check vectors are row and extract first mpc_N elements
    Fr_l0_mpc = Fr_l0(1:mpc_N);
    Fr_r0_mpc = Fr_r0(1:mpc_N);
    ref_com_mpc = ref_com(1:mpc_N);


    % single shooting
    [states, t] = computeRollout(state0, actual_t,mpc_dt, mpc_N, Fr_l0_mpc + delta_Fr_l, Fr_r0_mpc + delta_Fr_r,[0;0;0],int_method,int_steps);
 
    psi = states(1,:);
    l1 = states(2,:);
    l2 = states(3,:);
    psid = states(4,:);
    l1d = states(5,:);
    l2d = states(6,:); 
    [p, pd ]= computePositionVelocity(psi, l1, l2, psid,l1d, l2d);
    
    %p has mpc_N +1 elements 
    % track
    tracking = sum (vecnorm(ref_com_mpc - p).^2);
    
    % smoothnes: minimize jerky control action
    smooth = sum(diff(delta_Fr_l)) + sum(diff(delta_Fr_r));
         
    cost =  w1* tracking  + w2 *smooth ;
end