function cost = cost(x, p0,  pf)

    global m w1 w2 w3 w4 w5 w6  b  num_params N_dyn int_method  contact_normal

    Fleg = [ x(1); x(2); x(3)];
    Tf = x(4);
    Fr_l = x(num_params+1:num_params+N_dyn); 
    Fr_r = x(num_params+N_dyn+1:num_params+2*N_dyn); 

    % check they are column vectors
    p0 = p0(:);
    pf = pf(:);
    % variable intergration step
    dt_dyn = Tf / (N_dyn-1); 
    

    % single shooting
    state0 =  computeStateFromCartesian(p0);
    [~,~,states, t] = integrate_dynamics(state0,0, dt_dyn, N_dyn, Fr_l,Fr_r, Fleg,int_method);
    psi = states(1,:);
    l1 = states(2,:);
    l2 = states(3,:);
    psid = states(4,:);
    l1d = states(5,:);
    l2d = states(6,:); 
    [p, pd ]= computePositionVelocity(psi, l1, l2, psid,l1d, l2d);
    
    p_0 = p(:,1);
    p_f = p(:,end);

    % be careful there are only N values in this vector the path migh be
    % underestimated!
%     deltax = diff(p(1,:));  % diff(X);
%     deltay = diff(p(2,:));   % diff(Y);
%     deltaz = diff(p(3,:));    % diff(Z);
%     path_length = sum(sqrt(deltax.^2 + deltay.^2 + deltaz.^2));

    p_0 = p_0(:);
    p0 = p0(:);
    p_f= p_f(:);
    pf = pf(:);
    
    %minimize the final kin energy at contact
    Ekinfcost=  m/2*pd(:,end)'*pd(:,end);
      
    % minimize hoist work / energy consumption
    hoist_work = sum(abs(Fr_l.*l1d)*dt_dyn) + sum(abs(Fr_r.*l2d)*dt_dyn);  %assume the motor is not regenreating
    
    % smoothnes: minimize jerky control action
    smooth = sum(diff(Fr_r)) + sum(diff(Fr_l));
    
    %fprintf("hoist_work %f\n ",hoist_work)    
    %fprintf("smooth %f\n ", smooth)
    %fprintf("tempo %f\n ", w6*Tf)

     
    cost =   w4 *smooth ;% + w6*Tf;%
end