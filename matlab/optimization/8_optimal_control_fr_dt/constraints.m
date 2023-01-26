function [ineq, eq, number_of_constr, solution_constr] = constraints(x,   p0,  pf,  Fun_max, Fr_max, mu, int_steps )

global   m  num_params  T_th  N_dyn FRICTION_CONE  SUBSTEP_INTEGRATION int_method p_via


% ineq are <= 0
thetad0 = x(1);
phid0 = x(2);
Tf1 = x(3);
Tf2 = x(4);
Fr = x(num_params+1:num_params+2*N_dyn);        

% check they are column vectors
p0 = p0(:);
pf = pf(:);

% size not known
ineq = zeros(1,0);

% number of constraints
number_of_constr.wall_constraints = 2*N_dyn;
number_of_constr.retraction_force_constraints = 2*(2*N_dyn);
number_of_constr.initial_final_constraints = 3;

if FRICTION_CONE
    number_of_constr.force_constraints  = 3;
else
    number_of_constr.force_constraints  = 2;
end

% variable intergration step
dt_dyn1 = Tf1 / (N_dyn-1);
dt_dyn2 = Tf2 / (N_dyn-1);

% single shooting
[theta0, phi0, l_0] = computePolarVariables(p0);
state0 = [theta0, phi0, l_0, thetad0, phid0, 0];

%1 integrate and set dynamic constraints

if SUBSTEP_INTEGRATION
    %substep integraiton
    %before obstacle
    for i=1:N_dyn           
        if (i>=2)     
            [states(:,i), t(i)] = integrate_dynamics(states(:,i-1), t(i-1), dt_dyn1/(int_steps-1), int_steps, Fr(i-1)*ones(1,int_steps), int_method); % keep Fr constant           
        else
          states(:,i) = state0;
          t(i) = 0;      
        end    
    end
    %after obstacle
    %substep integraiton
    for i=N_dyn+1:2*N_dyn           
        [states(:,i), t(i)] = integrate_dynamics(states(:,i-1), t(i-1), dt_dyn2/(int_steps-1), int_steps, Fr(i-1)*ones(1,int_steps), int_method); % keep Fr constant           
    end
else
    % no substep integration
    %before obstacle
    [~,~,states(:, 1:N_dyn), t(1:N_dyn)] = integrate_dynamics(state0,0, dt_dyn1, N_dyn, Fr(1:N_dyn), int_method);
    % after obstacle
    [~,~,states(:, 1:N_dyn), t(1:N_dyn)] = integrate_dynamics(states(:,N_dyn),t(N_dyn), dt_dyn2, N_dyn, Fr(N_dyn+1:2*N_dyn), int_method);
end


theta = states(1,:);
phi = states(2,:);
l = states(3,:);
thetad = states(4,:);
phid = states(5,:);
ld = states(6,:);

p = [l.*sin(theta).*cos(phi); l.*sin(theta).*sin(phi); -l.*cos(theta)];
p_0 = p(:, 1);
p_f = p(:,end);
 

solution_constr.p = p;
solution_constr.theta = theta;
solution_constr.phi = phi;
solution_constr.l = l;
solution_constr.thetad = thetad;
solution_constr.phid = phid;
solution_constr.ld = ld;
solution_constr.time = t;
solution_constr.final_error_discrete = norm(p(:,end) - pf);



% 3 constraint to do not enter the wall, p_x >=0
for i=1:2*N_dyn 
    ineq = [ineq -p(1,i) ];   
    
end 
% % % debug
% disp('after wall')
% length(ineq)

% constraints on retraction force   -Fr_max < Fr = -K*(l-luncompr) < 0 
for i=1:2*N_dyn     
     ineq = [ineq  Fr(i) ]; % Fr <0    
end 
for i=1:2*N_dyn 
  ineq = [ineq -Fr_max - Fr(i)];    % -Fr_max -Fr <0
end

% debug
% disp('after Fr')
% length(ineq)

%evaluate inpulse ( the integral of the gaussian is 1) 
Fun = m*l_0*thetad0/T_th;
Fut = m*l_0*sin(theta0)*phid0/T_th;

ineq = [ineq  (sqrt(Fun^2 + Fut^2) -Fun_max)]   ;%(Fun < fun max )
ineq = [ineq  (-Fun)]  ; %(Fun >0 )
if FRICTION_CONE
    ineq = [ineq  (abs(Fut) -mu*Fun)]; %friction constraints
end
% 
% % debug
% disp('after Fu')
% length(ineq)


% via point  fixed slack 
% fixed_slack = 0.01*norm(p0 - p_via); 
% ineq= [ineq norm(p_via - p(:,N_dyn)) - fixed_slack];
% x_pos = 3;
% y_pos = 2;
% ineq= [ineq -p(1,N_dyn) +x_pos]; %px > x_pos
% ineq= [ineq -p(2,N_dyn) +y_pos]; %py > y_pos



% final point  fixed slack 
% fixed_slack = 0.02*norm(p0 - pf); 
% ineq= [ineq norm(p_f - pf) - fixed_slack];

% final point  variable slack  
ineq= [ineq norm(p_f - pf) - x(num_params+2*N_dyn+1)];

% stay below the anchor
for i=1:2*N_dyn 
    ineq = [ineq p(3,i) ];    
end 

eq = [];
if any(isinf(ineq))
    ineq
end


end