clc;
clear all;
close all;

% additional data for plotting
T_th   = 0.05; %on zeta variable
g = 9.81;
dt = 0.001;
m = 5;
Fun_max = 1000;
Fr_max = 130; % Fr in negative
mu = 0.8;
sigma_gauss = T_th/6;
mu_gauss    = T_th/2;
%%%%%%%%%%%%%%
l_0 = 3;
theta0 =atan2(0.38, l_0);
%theta0 = 0.05; 
phi0 = 0 ;
p0 = [l_0*sin(theta0)*cos(phi0); l_0*sin(theta0)*sin(phi0); -l_0*cos(theta0)];

% Marco Frego test: final state
pf = [0.001; 5.0; -8];
%
% additional data for plotting





T = table2array(readtable('30_08_22_1ms','NumHeaderLines',20)); % 21

zeta  = T(:, 2)';
fFun   = T(:, 17)';
fFut   = T(:, 16)';
Fr    = T(:, 7)';
theta = T(:,9)';
phi   = T(:,10)';
phid   = T(:,11)';
l     = T(:,12)';
thetad = T(:,13)';
ld     = T(:,14)';
Tf    = T(:,15)';
time  = (zeta.*Tf);
% get the impulse force
Fun = ForceFun(zeta,fFun,sigma_gauss,mu_gauss);
Fut = ForceFut(zeta,fFut,sigma_gauss,mu_gauss);



p = [l.*sin(theta).*cos(phi); l.*sin(theta).*sin(phi); -l.*cos(theta)]  ;

% velocity (variable length)
pd = [ld.*cos(phi).*sin(theta) - l.*phid.*sin(phi).*sin(theta) + l.*thetad.*cos(phi).*cos(theta);
ld.*sin(phi).*sin(theta) + l.*phid.*cos(phi).*sin(theta) + l.*thetad.*cos(theta).*sin(phi);
                                               l.*thetad.*sin(theta) - ld.*cos(theta)];


solution.energy = struct;
% init struct foc C++ code generation
solution.energy.Etot = 0;
solution.energy.Ekin = zeros(1, length(time));
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

for i =1:length(time)
    solution.energy.Ekin(i) = m/2*pd(:,i)'*pd(:,i);
    solution.energy.intEkin = solution.energy.intEkin +  solution.energy.Ekin(i)*dt;
end
    
%compare for sanity check should be equal to  E.Ekin0
solution.energy.Ekinfangles=  (m*l(end)^2/2).*(thetad(end)^2 + sin(theta(end))^2 *phid(end)^2) + m*ld(end)^2/2;
solution.energy.Ekinfx = m/2*pd(1,end)'*pd(1,end);
solution.energy.Ekinfy = m/2*pd(2,end)'*pd(2,end);
solution.energy.Ekinfz = m/2*pd(3,end)'*pd(3,end);
solution.energy.Ekinf = m/2*pd(:,end)'*pd(:,end);
solution.initial_error = norm(p(:,1) -p0);
solution.final_error_real = norm(p(:,end) -pf);

solution.p = p;
solution.theta = theta;
solution.phi = phi;
solution.l = l;
solution.thetad = thetad;
solution.phid = phid;
solution.ld = ld;
solution.time = time;
solution.zeta = zeta';
%evaluate inpulse ( the integral of the gaussian is 1) 
solution.Fun = Fun;
solution.Fut = Fut;
solution.Fr = Fr;

figure
plot(time, Fun)
ylabel('Fun')

figure
plot(time, Fut)
ylabel('Fut')
figure
plot_curve(solution, p0, pf,  'r');
save('test_marco.mat','solution','T_th','mu','Fun_max', 'Fr_max', 'p0','pf');



function res = ForceFun(zeta,Fun0,sigma,mu_gauss)

  res =  Fun0.*1/sqrt(2*pi*sigma^2).*exp(-(zeta-mu_gauss).^2./(2*sigma^2));
end

function res = ForceFut(zeta,Fut0,sigma,mu_gauss)
  res =  Fut0.*1/sqrt(2*pi*sigma^2).*exp(-(zeta-mu_gauss).^2./(2*sigma^2));
  
end

