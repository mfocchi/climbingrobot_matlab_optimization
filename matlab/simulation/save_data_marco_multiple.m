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

pf_vec = [];
exp_name = {'exp1';'exp2';'exp3';'exp4'};

%exp 2          p0 = [ 0; 0 ; -8]           pf =[2.60  2.73  -20.46]
%exp3           p0 = [ 0; 0 ; -8]            pf =[ 2.50  0.05   -20.46]
%exp5           p0 = [ 0; 0 ; -8]           pf = [0.55;  -0.8;  -17.76]
%exp 6 slanted  p0 = [0.63 , 2.35; -7.5],	pf = [1.734, 3.8, -20.46]

for n_test =1:size(exp_name,1)

    
T = table2array(readtable(exp_name{n_test},'NumHeaderLines',20)); 


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

%0.04198*2/1.698444
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



plot_curve( solution, p0, pf, mu,  'r', n_test==1);hold on;
pf_vec = [pf_vec; pf'];

%ad obstacle
cone(0,1.5,0);

save(exp_name{n_test},'solution','T_th','mu','Fun_max', 'Fr_max', 'p0','pf');

n_test
Tf(end)
Tth_index = min(find(time>=T_th));
Fun(floor(Tth_index/2))
Fut(floor(Tth_index/2))
solution.energy.Ekinf
norm(p0-pf)
end

view(113,61); 
text(pf_vec(1,1)+1,pf_vec(1,2)-1,pf_vec(1,3), 'Exp.1','FontSize',20)
text(pf_vec(2,1),pf_vec(2,2)-2,pf_vec(2,3), 'Exp.2','FontSize',20)
text(pf_vec(3,1)+1,pf_vec(3,2)-2,pf_vec(3,3), 'Exp.3','FontSize',20)
text(pf_vec(4,1)+1,pf_vec(4,2)-0.5,pf_vec(4,3), 'Exp.4','FontSize',20)



%save the plot
set(gcf, 'Paperunits' , 'centimeters')
set(gcf, 'PaperSize', [20 27]);
set(gcf, 'PaperPosition', [0 0 20 27]);
print(gcf, '-dpdf',strcat('../../paper/matlab/targets.pdf'),'-painters')



function res = ForceFun(zeta,Fun0,sigma,mu_gauss)

  res =  Fun0.*1/sqrt(2*pi*sigma^2).*exp(-(zeta-mu_gauss).^2./(2*sigma^2));
end

function res = ForceFut(zeta,Fut0,sigma,mu_gauss)
  res =  Fut0.*1/sqrt(2*pi*sigma^2).*exp(-(zeta-mu_gauss).^2./(2*sigma^2));
  
end

