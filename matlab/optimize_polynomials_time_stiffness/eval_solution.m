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
theta = a_10 + a_11*t + a_12*t.^2 +  a_13*t.^3;
thetad =  a_11 + 2*a_12*t + 3*a_13*t.^2;
thetadd = 2*a_12 + 6*a_13*t;

phi = a_20 + a_21*t + a_22*t.^2 + a_23*t.^3;
phid =  a_21 + 2*a_22*t  + 3*a_23*t.^2;
phidd =   2*a_22 + 6*a_23*t;

l = a_30 + a_31*t + a_32*t.^2 + a_33*t.^3;
ld =  a_31 + 2*a_32*t  + 3*a_33*t.^2; 

p = [l.*sin(theta).*cos(phi); l.*sin(theta).*sin(phi); -l.*cos(theta)]  ;

%disp(strcat('Initial velocity is [theta0, phi0]:',num2str(thetad(1)),"   ", num2str(phid(1))) );


% velocity
pd = [cos(theta).*cos(phi).*thetad.*l - sin(phi).*sin(theta).*phid.*l;
    cos(theta).*sin(phi).*thetad .*l + cos(phi).*sin(theta).*phid.*l;
    sin(theta).*thetad.*l];

deltax = diff(p(1,:));  % diff(X);
deltay = diff(p(2,:));   % diff(Y);
deltaz = diff(p(3,:));    % diff(Z);
path_length = sum(sqrt(deltax.^2 + deltay.^2 + deltaz.^2));

% check length is always l
%     a = vecnorm(p)
%     a -  ones(1,length(a))*l


E = struct;


% Calculating and ploting the total Energy from the new fit: theta, thetad and phid
E.Etot =  ((m*l.^2/2).*(thetad.^2 + sin(theta).^2 .*phid.^2)) +m.*ld.^2/2 - m*g*l.*cos(theta) + K*(l-l_uncompressed).^2/2;



% kinetic energy at the beginning
E.Ekin0x = m/2*pd(1,1)'*pd(1,1);
E.Ekin0y = m/2*pd(2,1)'*pd(2,1);
E.Ekin0z = m/2*pd(3,1)'*pd(3,1);
E.Ekin0 = m/2*pd(:,1)'*pd(:,1);

%compare for sanity check should be equal to  E.Ekin0
%E.Ekin0angles=  (m*l^2/2).*(thetad(1)^2 + sin(theta(1))^2 *phid(1)^2);

E.U0 =  -m*g*l*cos(theta(1)) + K*(l(1)-l_uncompressed).^2/2;

E.Ekinfx = m/2*pd(1,end)'*pd(1,end);
E.Ekinfy = m/2*pd(2,end)'*pd(2,end);
E.Ekinfz = m/2*pd(3,end)'*pd(3,end);
E.Ekinf = m/2*pd(:,end)'*pd(:,end);
E.Uf = -m*g*l*cos(theta(end)) +K*(l(end)-l_uncompressed).^2/2;

initial_error = norm(p(:,1) -p0);
final_error = norm(p(:,end) -pf);

end