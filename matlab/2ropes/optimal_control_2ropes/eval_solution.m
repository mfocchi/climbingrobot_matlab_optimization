function solution = eval_solution(x,  dt, p0, pf)

global m g   num_params  N_dyn  int_method
%eval trajectory
Fleg = [ x(1); x(2); x(3)];
Tf = x(4);
Fr_l = x(num_params+1:num_params+N_dyn); 
Fr_r = x(num_params+N_dyn+1:num_params+2*N_dyn); 

dt_dyn = Tf / (N_dyn-1); 
% single shooting
x0 =  computeStateFromCartesian(p0);
[~,~,x, t] = integrate_dynamics(x0,0, dt_dyn, N_dyn, Fr_l,Fr_r, Fleg,int_method);
psi = x(1,:);
l1 = x(2,:);
l2 = x(3,:);
psid = x(4,:);
l1d = x(5,:);
l2d = x(6,:);

[p, pd ]= computePositionVelocity(psi, l1, l2, psid,l1d, l2d);
   

p_0 = p(:, 1);
p_f = p(:,end);

%disp(strcat('Initial velocity is [theta0, phi0]:',num2str(thetad(1)),"   ", num2str(phid(1))) );
% velocity (constant length)
% pd = [cos(theta).*cos(phi).*thetad.*l - sin(phi).*sin(theta).*phid.*l;
%     cos(theta).*sin(phi).*thetad .*l + cos(phi).*sin(theta).*phid.*l;
%     sin(theta).*thetad.*l];


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
solution.energy.Ekinfx = 0;
solution.energy.Ekinfy = 0;
solution.energy.Ekinfz = 0;
solution.energy.Ekinf = 0;
solution.energy.Uf = 0;

% Calculating and ploting the total Energy from the new fit: theta, thetad and phid
%TODO solution.energy.Etot =  (m*l.^2/2).*(thetad.^2 + sin(theta).^2 .*phid.^2) +m.*ld.^2/2 - m*g*l.*cos(theta);


% kinetic energy at the beginning
solution.energy.Ekin0x = m/2*pd(1,1)'*pd(1,1);
solution.energy.Ekin0y = m/2*pd(2,1)'*pd(2,1);
solution.energy.Ekin0z = m/2*pd(3,1)'*pd(3,1);
solution.energy.Ekin0 = m/2*pd(:,1)'*pd(:,1);


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