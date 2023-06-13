function solution = eval_solution(x,  dt, p0, pf)

global m g   num_params  N_dyn  int_method int_steps
%eval trajectory
Fleg = [ x(1); x(2); x(3)];
Tf = x(4);
Fr_l = x(num_params+1:num_params+N_dyn); 
Fr_r = x(num_params+N_dyn+1:num_params+2*N_dyn); 

dt_dyn = Tf / (N_dyn-1); 
% single shooting
state0 =  computeStateFromCartesian(p0);
[states, t] = computeRollout(state0, 0,dt_dyn, N_dyn, Fr_l, Fr_r,Fleg,int_method,int_steps);
psi = states(1,:);
l1 = states(2,:);
l2 = states(3,:);
psid = states(4,:);
l1d = states(5,:);
l2d = states(6,:); 
[p, pd ]= computePositionVelocity(psi, l1, l2, psid,l1d, l2d);
   


p_0 = p(:, 1);
p_f = p(:,end);

deltax = diff(p(1,:));  % diff(X);
deltay = diff(p(2,:));   % diff(Y);
deltaz = diff(p(3,:));    % diff(Z);
solution.path_length = sum(sqrt(deltax.^2 + deltay.^2 + deltaz.^2));

% check length is always l
%     a = vecnorm(p)
%     a -  ones(1,length(a))*l


solution.energy = struct;
% init struct foc C++ code generation
solution.energy.Etot = 0;
solution.energy.Ekin = zeros(1, length(t));
solution.energy.Ekin0x = 0;
solution.energy.Ekin0y = 0;
solution.energy.Ekin0z = 0;
solution.energy.Ekin0 = 0;
solution.energy.intEkin = 0;
solution.energy.U0 = 0;
solution.energy.Uf = 0;

% kinetic energy at the beginning
solution.energy.Ekin0x = m/2*pd(1,1)'*pd(1,1);
solution.energy.Ekin0y = m/2*pd(2,1)'*pd(2,1);
solution.energy.Ekin0z = m/2*pd(3,1)'*pd(3,1);
solution.energy.Ekin0 = m/2*pd(:,1)'*pd(:,1);

solution.energy.Ekinfx = m/2*pd(1,end)'*pd(1,end);
solution.energy.Ekinfy = m/2*pd(2,end)'*pd(2,end);
solution.energy.Ekinfz = m/2*pd(3,end)'*pd(3,end);
solution.energy.Ekinf = m/2*pd(:,end)'*pd(:,end);


for i =1:length(t)
    solution.energy.Ekin(i) = m/2*pd(:,i)'*pd(:,i);
    solution.energy.intEkin = solution.energy.intEkin +  solution.energy.Ekin(i)*dt;
end
    
solution.initial_error = norm(p(:,1) -p0);
solution.final_error_real = norm(p(:,end) -pf);

solution.Fleg = Fleg;
solution.Fr_l  = Fr_l;
solution.Fr_r  = Fr_r;
solution.p = p;
solution.psi = psi;
solution.l1 = l1;
solution.l2 = l2;
solution.psid = psid;
solution.l1d = l1d;
solution.l2d = l2d;
solution.time = t;
solution.Tf = Tf;



solution.achieved_target =  p(:,end);

end