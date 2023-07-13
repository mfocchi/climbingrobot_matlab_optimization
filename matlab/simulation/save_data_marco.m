clc;
clear all;
close all;

% additional data for plotting
T_th   = 0.05; %on zeta variable
g = 9.81;
dt = 0.001;
m = 5;
Fun_max = 1000;
Fr_max = 200; % Fr in negative
mu = 0.8;
sigma_gauss = T_th/6;
mu_gauss    = T_th/2;
%%%%%%%%%%%%%%
l_0 = 3;
theta0 =atan2(0.38, l_0);
%theta0 = 0.05; 
phi0 = 0 ;
%p0 = [l_0*sin(theta0)*cos(phi0); l_0*sin(theta0)*sin(phi0); -l_0*cos(theta0)];
% Marco Frego test: final state
%pf = [0.3777; 1.5; -20]

%T = table2array(readtable('08_09_2022_obstacle','NumHeaderLines',20)); % 21
%T = table2array(readtable('09_09_2022mu07','NumHeaderLines',20)); % 21
%T = table2array(readtable('09_09_2022_3','NumHeaderLines',20)); % obstacle (0, 1.5)
T = table2array(readtable('first_jump_0224_0_8_to_3_3_20','NumHeaderLines',20)); % obstacle (0, 1.5) , pf = [3,3,-20] 1st jump
%T = table2array(readtable('second_jump_3_3_20_to_0224_5_25','NumHeaderLines',20)); % obstacle (0, 1.5) , p0 = [0.,3,-20] 2nd jump

% prin jump up from target
%T = table2array(readtable('up_jump_3_3_20_to_0224_0_8','NumHeaderLines',20)); % obstacle (0, 1.5) , p0 = [3,3,-20] pf = [0.0224 0, -8] 



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



p0 = p(:,1);
pf = p(:,end);

% test if inside cone
% p0 = [0.2240;         0;   -8.0000];
% pf = [3;        3;   -20.0000];

T_th = Tf(1)*0.05;

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
solution.Tf = Tf(end);
%evaluate inpulse ( the integral of the gaussian is 1) 
solution.Fun = Fun;
solution.Fut = Fut;
solution.Fr = Fr;


%%Energy consumption
J_TO_Wh = 0.000277 %maps joule to Wh
impulse_end_idx = max(find(time<=T_th))
impulse_work =   m/2*pd(:,impulse_end_idx)'*pd(:,impulse_end_idx);
hoist_work = 0;
for i=1:length(time)
    hoist_work = hoist_work + abs(Fr(impulse_end_idx).*ld(impulse_end_idx))* dt;  %assume the motor is not regenreating
end
impulse_workWh = J_TO_Wh*impulse_work
hoist_workWh=J_TO_Wh*hoist_work




figure
plot_curve( solution, p0, pf, mu,  'r');
%ad obstacle
cone(0,1.5,0);
%cone(0,5,-4)
% 
figure
plot(time, Fun)
ylabel('Fun')

figure
plot(time, Fut)
ylabel('Fut')

figure
plot(time, Fr)
ylabel('Fr')

% friction coeff
% figure
% plot(time, fFut./fFun)


% test of the leg displacement 
% r_leg= 0.32;
% p0_real = p0 + computeNormalRope(r_leg,[0;0;0], p0)*r_leg;
% pf_real = pf + computeNormalRope(r_leg,[0;0;0], pf)*r_leg;
% plot3(p0_real(1), p0_real(2), p0_real(3), 'Marker', '.', 'Color','g', 'MarkerSize',60) ;
% plot3(pf_real(1), pf_real(2), pf_real(3), 'Marker', '.', 'Color','r', 'MarkerSize',60) ;
% arrow3d_points(p0, p0_real, 'stemWidth',0.02,'color','r')   ;
% arrow3d_points(pf,  pf_real, 'stemWidth',0.02,'color','r')  ;                                       


save('test_optim_marco.mat','solution','T_th','mu','Fun_max', 'Fr_max', 'p0','pf');


function res = ForceFun(zeta,Fun0,sigma,mu_gauss)

  res =  Fun0.*1/sqrt(2*pi*sigma^2).*exp(-(zeta-mu_gauss).^2./(2*sigma^2));
end

function res = ForceFut(zeta,Fut0,sigma,mu_gauss)
  res =  Fut0.*1/sqrt(2*pi*sigma^2).*exp(-(zeta-mu_gauss).^2./(2*sigma^2));
  
end

