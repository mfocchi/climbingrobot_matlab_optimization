function coste = cost(x, p0,  pf, int_steps, fixed_time)

    global m w1 w2 w3 w4 w5 w6 N  T_th num_params N_dyn  

    thetad0 = x(1);
    phid0 = x(2);
    K = x(3);
    l_uncompressed = x(4);
   
    switch nargin
        case 5
            Tf = fixed_time;
            %fprintf(2, 'cost: time optim off\n')
        otherwise           
            Tf = x(5);
    end

    % variable intergration step
    dt_dyn = Tf / N_dyn; 
    
    %accurate intergation
%     [theta0, phi0, l_0] = computePolarVariables(p0);
%     state0 = [theta0, phi0, l_0, thetad0, phid0, 0];
%     %1 integrate dynamics
%     for i=1:N_dyn          
%         if (i>=2)
%           [states(:,i), t(i)] = integrate_dynamics(states(:,i-1), t(i-1), dt_dyn/(int_steps-1), int_steps, K,l_uncompressed);          
%         else
%           states(:,i) = state0;
%           t(i) = 0;  
%         end
%     end

    %rough integration
    [theta0, phi0, l_0] = computePolarVariables(p0);
    state0 = [theta0, phi0, l_0, thetad0, phid0, 0];
    [states, t] = integrate_dynamics(state0,0, dt_dyn, N_dyn, K,l_uncompressed, 'rk4');



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
    
    % energy as ineq constraint
    slack_energy=  sum(x(num_params+1:num_params+N)); 
    %energy as eq constrraints 
    %slack_energy= max(abs(x(num_params+1:num_params+N)));

    slack_dyn = max(x(num_params+N+1:num_params+N+N_dyn));
    slack_final  = max (x(num_params+N+N_dyn + 1:end));

    Ekinfcost=  ( (m*l(end)^2/2).*( thetad(end)^2 + sin(theta(end))^2 *phid(end)^2 )  + (m*ld(end)^2/2)   );
    Fut = m*l_0*sin(theta0)*phid0/T_th;
    
    %final = w4 * slack_final
    %energy = w3 * slack_energy
    %ekinf= w5* Ekinfcost
    %dyn= w6* slack_dyn
    
    % fut = abs(Fut)  % minimizing this and increasing the weight it turns
    % the trajectory vertical 

    
    %coste =  w3 * slack_energy + w5* Ekinfcost +  w6* slack_dyn  +w4 * slack_final;
    coste =    w5* Ekinfcost;

end