function [p, theta, phi, l,  E,  path_length, initial_error, final_error, thetad, phid,ld, t] = eval_solution(x,  dt, p0, pf)

global m g l_uncompressed N_dyn dt_dyn
%eval trajectory
thetad0 = x(1);
phid0 = x(2);
K = x(3);
% Tf = x(4);


[theta0, phi0, l_0] = computePolarVariables(p0);
state0 = [theta0, phi0, l_0, thetad0, phid0, 0];
[states, t] = integrate_dynamics(state0,dt_dyn, N_dyn, K);

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
path_length = sum(sqrt(deltax.^2 + deltay.^2 + deltaz.^2));

% check length is always l
%     a = vecnorm(p)
%     a -  ones(1,length(a))*l


E = struct;
% init struct foc C++ code generation
E.Etot = 0;
E.Ekin = zeros(1, length(t));
E.Ekin0x = 0;
E.Ekin0y = 0;
E.Ekin0z = 0;
E.Ekin0 = 0;
E.intEkin = 0;
E.U0 = 0;
E.Ekinfx = 0;
E.Ekinfy = 0;
E.Ekinfz = 0;
E.Ekinf = 0;
E.Uf = 0;

% Calculating and ploting the total Energy from the new fit: theta, thetad and phid
E.Etot =  (m*l.^2/2).*(thetad.^2 + sin(theta).^2 .*phid.^2) +m.*ld.^2/2 - m*g*l.*cos(theta) + K*(l-l_uncompressed).^2/2;


% kinetic energy at the beginning
E.Ekin0x = m/2*pd(1,1)'*pd(1,1);
E.Ekin0y = m/2*pd(2,1)'*pd(2,1);
E.Ekin0z = m/2*pd(3,1)'*pd(3,1);
E.Ekin0 = m/2*pd(:,1)'*pd(:,1);


for i =1:length(t)
    E.Ekin(i) = m/2*pd(:,i)'*pd(:,i);
    E.intEkin = E.intEkin +  E.Ekin(i)*dt;
end
    
%compare for sanity check should be equal to  E.Ekin0
E.Ekinfangles=  (m*l(end)^2/2).*(thetad(end)^2 + sin(theta(end))^2 *phid(end)^2) + m*ld(end)^2/2;

E.U0 =  -m*g*l(1)*cos(theta(1)) + K*(l(1)-l_uncompressed).^2/2;
E.Ekinfx = m/2*pd(1,end)'*pd(1,end);
E.Ekinfy = m/2*pd(2,end)'*pd(2,end);
E.Ekinfz = m/2*pd(3,end)'*pd(3,end);
E.Ekinf = m/2*pd(:,end)'*pd(:,end);
E.Uf = -m*g*l(end)*cos(theta(end)) +K*(l(end)-l_uncompressed).^2/2;

initial_error = norm(p(:,1) -p0);
final_error = norm(p(:,end) -pf);

end