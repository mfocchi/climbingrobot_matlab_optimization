function coste = cost(x, p0,  pf, int_steps)

    global m w1 w2 w3 w4 w5 w6 N  T_th num_params N_dyn int_method   SUBSTEP_INTEGRATION

    thetad0 = x(1);
    phid0 = x(2);
    Tf1 = x(3);
    Tf2 = x(4);
    Fr = x(num_params+1:num_params+2*N_dyn);        

   
    % variable intergration step
    dt_dyn1 = Tf1 / (N_dyn-1);
    dt_dyn2 = Tf2 / (N_dyn-1);
    
    %accurate intergation
    [theta0, phi0, l_0] = computePolarVariables(p0);
    state0 = [theta0, phi0, l_0, thetad0, phid0, 0];
    
%     
%     if SUBSTEP_INTEGRATION
%         %substep integraiton
%         %before obstacle
%         for i=1:N_dyn           
%             if (i>=2)     
%                 [states(:,i), t(i)] = integrate_dynamics(states(:,i-1), t(i-1), dt_dyn1/(int_steps-1), int_steps, Fr(i-1)*ones(1,int_steps), int_method); % keep Fr constant           
%             else
%               states(:,i) = state0;
%               t(i) = 0;      
%             end    
%         end
%         %after obstacle
%         %substep integraiton
%         for i=N_dyn+1:2*N_dyn           
%             [states(:,i), t(i)] = integrate_dynamics(states(:,i-1), t(i-1), dt_dyn2/(int_steps-1), int_steps, Fr(i-1)*ones(1,int_steps), int_method); % keep Fr constant           
%         end
%     else
        % no substep integration
        %before obstacle
        [~,~,states(:, 1:N_dyn), t(1:N_dyn)] = integrate_dynamics(state0,0, dt_dyn1, N_dyn, Fr(1:N_dyn), int_method);
        % after obstacle
        [~,~,states(:, 1:N_dyn), t(1:N_dyn)] = integrate_dynamics(states(:,N_dyn),t(N_dyn), dt_dyn2, N_dyn, Fr(N_dyn+1:2*N_dyn), int_method);
   % end



    theta = states(1,:);
    phi = states(2,:);
    l = states(3,:);
    thetad = states(4,:);
    phid = states(5,:);
    ld = states(6,:);
    
    p = [l.*sin(theta).*cos(phi); l.*sin(theta).*sin(phi); -l.*cos(theta)];
    p_0 = p(:,1);
    p_f = p(:,end);
    l_f = l(end);
    

    % be careful there are only N values in this vector the path migh be
    % underestimated!
    deltax = diff(p(1,:));  % diff(X);
    deltay = diff(p(2,:));   % diff(Y);
    deltaz = diff(p(3,:));    % diff(Z);
    path_length = sum(sqrt(deltax.^2 + deltay.^2 + deltaz.^2));
    p_0 = p_0(:);
    p0 = p0(:);
    p_f= p_f(:);
    pf = pf(:);
    
   
    Ekinfcost=  ( (m*l(end)^2/2).*( thetad(end)^2 + sin(theta(end))^2 *phid(end)^2 )  + (m*ld(end)^2/2)   );
    Fut = m*l_0*sin(theta0)*phid0/T_th;

    slack_final  = sum (x(num_params+2*N_dyn + 1:end));
    % fut = abs(Fut)  % minimizing this and increasing the weight it turns
%+ w4 * max(diff(Fr))  +  w5* Ekinfcost
    
    %coste =  w3 * slack_energy + w5* Ekinfcost +  w6* slack_dyn  +w4 * slack_final;
    coste =   0.001*abs(Fut) +w4 * slack_final;

end