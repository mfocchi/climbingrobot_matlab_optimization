function [p, E,  path_length, initial_error, final_error] = eval_solution(x,  dt, p0, pf)

global m g l_uncompressed
%eval trajectory

Tf = x(1);
t = linspace(0, Tf, 1/dt);
a_10 = x(2);
a_11 = x(3);
a_12 = x(4);
a_13 = x(5);
a_20 = x(6);
a_21 = x(7);
a_22 = x(8);
a_23 = x(9);
a_30 = x(10);
a_31 = x(11);
a_32 = x(12);
a_33 = x(13);
K = x(14);

% parametrizzation with sin theta sing phi
arg1 = a_10 + a_11*t + a_12*t.^2 +  a_13*t.^3;
arg1d = a_11 + 2*a_12*t + 3*a_13*t.^2;
s_theta = arg1;
c_theta = sqrt(1 -  s_theta.^2);
arg2 = a_20 + a_21*t + a_22*t.^2 + a_23*t.^3;
arg2d =  a_21 + 2*a_22*t  + 3*a_23*t.^2;
s_phi = arg2;
c_phi = sqrt(1 -  s_phi.^2);
thetad = 1./sqrt(1-arg1.^2).*arg1d;
phid = 1./sqrt(1-arg2.^2).*arg2d;

l = a_30 + a_31*t + a_32*t.^2 + a_33*t.^3;
ld =  a_31 + 2*a_32*t  + 3*a_33*t.^2; 

p = [l.*s_theta.*c_phi; l.*s_theta.*s_phi; -l.*c_theta];
%disp(strcat('Initial velocity is [theta0, phi0]:',num2str(thetad(1)),"   ", num2str(phid(1))) );


% velocity
pd = [c_theta.*c_phi.*thetad.*l - s_phi.*s_theta.*phid.*l;
    c_theta.*s_phi.*thetad .*l + c_phi.*s_theta.*phid.*l;
    s_theta.*thetad.*l];

deltax = diff(p(1,:));  % diff(X);
deltay = diff(p(2,:));   % diff(Y);
deltaz = diff(p(3,:));    % diff(Z);
path_length = sum(sqrt(deltax.^2 + deltay.^2 + deltaz.^2));

% check length is always l
%     a = vecnorm(p)
%     a -  ones(1,length(a))*l


E = struct;


% Calculating and ploting the total Energy from the new fit: theta, thetad and phid
E.Etot = m*l.^2/2.*(thetad.^2 + s_theta.^2 .*phid.^2) + m*ld.^2/2 - m*g*l.*c_theta + K*(l-l_uncompressed).^2/2;

% kinetic energy at the beginning
E.Ekin0x = m/2*pd(1,1)'*pd(1,1);
E.Ekin0y = m/2*pd(2,1)'*pd(2,1);
E.Ekin0z = m/2*pd(3,1)'*pd(3,1);
E.Ekin0 = m/2*pd(:,1)'*pd(:,1);

%compare for sanity check should be equal to  E.Ekin0
%E.Ekin0angles=  (m*l^2/2).*(thetad(1)^2 + sin(theta(1))^2 *phid(1)^2);

E.U0 =  -m*g*l*c_theta(1) + K*(l(1)-l_uncompressed).^2/2;

E.Ekinfx = m/2*pd(1,end)'*pd(1,end);
E.Ekinfy = m/2*pd(2,end)'*pd(2,end);
E.Ekinfz = m/2*pd(3,end)'*pd(3,end);
E.Ekinf = m/2*pd(:,end)'*pd(:,end);
E.Uf = -m*g*l*c_theta(end) +K*(l(end)-l_uncompressed).^2/2;

initial_error = norm(p(:,1) -p0);
final_error = norm(p(:,end) -pf);

end