function solution = eval_solution(x,  dt, p0, pf)

global m g   num_params  N_dyn T_th int_method
%eval trajectory
thetad0 = x(1);
phid0 = x(2);
Tf1 = x(3);
Tf2 = x(4);

Fr_rough = x(num_params+1:num_params+2*N_dyn); 
%resample Fr rough that now is sampled in 2 parts of different dt
n_samples1 = floor(Tf1/dt);
rough_count = 1;
t_ = 0;

for i=1: n_samples1

    
   t_= t_+dt;
   if t_> ((n_samples1) *dt/(N_dyn))
        rough_count = rough_count + 1;
        t_ =0;
        
   end  
  
    Fr(i) =  Fr_rough(rough_count);
end

n_samples2 = floor(Tf2/dt);
t_ = 0;
% you need to do the first transition yourself cause rough_count is Ndyn
% and you arleady filled in that step
rough_count = N_dyn +1;
for i=n_samples1+1:n_samples1 + n_samples2
   t_= t_+dt;   
   if t_> ((n_samples2) *dt/(N_dyn))
        rough_count = rough_count + 1;
        t_ =0;
        
   end      
    Fr(i) =  Fr_rough(rough_count);
end

solution.Fr_rough  = Fr_rough;
solution.Fr  = Fr;
solution.Tf1 = Tf1;
solution.Tf2 = Tf2;

[theta0, phi0, l_0] = computePolarVariables(p0);
state0 = [theta0, phi0, l_0, thetad0, phid0, 0];
%tried with (Tf1+Tf2)/(n_samples1+n_samples2 it does not change
[~,~,states, t] = integrate_dynamics(state0,0,dt, n_samples1+n_samples2, Fr);


theta = states(1,:);
phi = states(2,:);
l = states(3,:);
thetad = states(4,:);
phid = states(5,:);
ld = states(6,:);

% check they are column vectors
pf = pf(:);
p0 = p0(:);


p = [l.*sin(theta).*cos(phi); l.*sin(theta).*sin(phi); -l.*cos(theta)]  ;

%disp(strcat('Initial velocity is [theta0, phi0]:',num2str(thetad(1)),"   ", num2str(phid(1))) );
% velocity (constant length)
% pd = [cos(theta).*cos(phi).*thetad.*l - sin(phi).*sin(theta).*phid.*l;
%     cos(theta).*sin(phi).*thetad .*l + cos(phi).*sin(theta).*phid.*l;
%     sin(theta).*thetad.*l];

% velocity (variable length)
pd = [ld.*cos(phi).*sin(theta) - l.*phid.*sin(phi).*sin(theta) + l.*thetad.*cos(phi).*cos(theta);
ld.*sin(phi).*sin(theta) + l.*phid.*cos(phi).*sin(theta) + l.*thetad.*cos(theta).*sin(phi);
                                               l.*thetad.*sin(theta) - ld.*cos(theta)];

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
solution.energy.Etot =  (m*l.^2/2).*(thetad.^2 + sin(theta).^2 .*phid.^2) +m.*ld.^2/2 - m*g*l.*cos(theta);


% kinetic energy at the beginning
solution.energy.Ekin0x = m/2*pd(1,1)'*pd(1,1);
solution.energy.Ekin0y = m/2*pd(2,1)'*pd(2,1);
solution.energy.Ekin0z = m/2*pd(3,1)'*pd(3,1);
solution.energy.Ekin0 = m/2*pd(:,1)'*pd(:,1);


for i =1:length(t)
    solution.energy.Ekin(i) = m/2*pd(:,i)'*pd(:,i);
    solution.energy.intEkin = solution.energy.intEkin +  solution.energy.Ekin(i)*dt;
end
    
%compare for sanity check should be equal to  E.Ekin0
solution.energy.Ekinfangles=  (m*l(end)^2/2).*(thetad(end)^2 + sin(theta(end))^2 *phid(end)^2) + m*ld(end)^2/2;

solution.energy.U0 =  -m*g*l(1)*cos(theta(1));
solution.energy.Ekinfx = m/2*pd(1,end)'*pd(1,end);
solution.energy.Ekinfy = m/2*pd(2,end)'*pd(2,end);
solution.energy.Ekinfz = m/2*pd(3,end)'*pd(3,end);
solution.energy.Ekinf = m/2*pd(:,end)'*pd(:,end);
solution.energy.Uf = -m*g*l(end)*cos(theta(end))  ;

solution.initial_error = norm(p(:,1) -p0);
solution.final_error_real = norm(p(:,end) -pf);


solution.p = p;
solution.theta = theta;
solution.phi = phi;
solution.l = l;
solution.thetad = thetad;
solution.phid = phid;
solution.ld = ld;
solution.time = t;

%evaluate inpulse ( the integral of the gaussian is 1) 
solution.Fun = m*l_0*thetad(1)/T_th;
solution.Fut = m*l_0*sin(theta(1))*phid(1)/T_th;


end