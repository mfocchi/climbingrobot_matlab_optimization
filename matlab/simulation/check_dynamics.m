clc;
clear all;
close all;

% additional data for plotting
m     = 5;
Tth   = 0.05; %on zeta variable
Fun_max = 600;
g = 9.81;
%%%%%%%%%%%%%%

T = table2array(readtable('Rocciatore_OCP_result','NumHeaderLines',20)); % 21

zeta  = T(:, 2);
Fun   = T(:, 17);
Fut   = T(:, 16);
Fr    = T(:, 7);
theta = T(:,9);
phi   = T(:,10);
psi   = T(:,11);
l     = T(:,12);
omega = T(:,13);
r     = T(:,14);
Tf    = T(:,15);
time  = zeta.*Tf;
KinEnergy = m*l.^2/2.*(omega.^2+sin(theta).^2.*psi.^2) + m/2*r.^2;

X     = l.*sin(theta).*cos(phi);
Y     = l.*sin(theta).*sin(phi);
Z     = -l.*cos(theta);

% integrate dynamics
theta_i(1) = theta(1);
phi_i(1) = phi(1);
l_i(1) = l(1);
dtheta_i(1) = omega(1);
dphi_i(1) = psi(1);
dl_i(1) = r(1);


dt = 0.001 * Tf(1);
npts = size(zeta,1); 

% get the impulse force in zeta
fFun = ForceFun(zeta,Fun,Tth);% remeber to scale Th
fFut = ForceFut(zeta,Fut,Tth);% remeber to scale Th


for i=2:npts
    
%     theta_i(i) = theta_i(i-1) + dt*(omega(i));
%     phi_i(i) = phi_i(i-1) + dt*(psi(i));
%     l_i(i) = l_i(i-1) + dt*(r(i));
    ddtheta = -(2*omega.*r)./l + cos(theta).*sin(theta).*(psi.^2)-(g./l).*sin(theta)+fFun./(m.*l);
    ddphi = -2*(cos(theta)./sin(theta)).*psi.*omega-(2./l).*psi.*r+ fFut./(m*l.*sin(theta));
    ddl = l.*(omega.^2)+l.*(sin(theta).^2).*psi.^2+g*cos(theta)+(1/m)*Fr;
    
    dtheta_i(i) = dtheta_i(i-1) + dt*(ddtheta(i));
    dphi_i(i) = dphi_i(i-1) + dt*(ddphi(i));
    dl_i(i) = dl_i(i-1) + dt*(ddl(i));
%     
    theta_i(i) = theta_i(i-1) + dt*(dtheta_i(i)) + 0.5 *dt^2.*ddtheta(i);
    phi_i(i) = phi_i(i-1) + dt*(dphi_i(i)) + 0.5 *dt^2.*ddphi(i);
    l_i(i) = l_i(i-1) + dt*(dl_i(i)) + 0.5 *dt^2.*ddl(i);
     
end

X_i     = l_i.*sin(theta_i).*cos(phi_i);
Y_i     = l_i.*sin(theta_i).*sin(phi_i);
Z_i     = -l_i.*cos(theta_i);

figure
subplot(3,1,1)
plot(time,theta_i); hold on;grid on;
plot(time,theta,'r')
ylabel('theta')
legend("integrated", "normal");

subplot(3,1,2)
plot(time, phi_i); hold on;grid on;
plot(time, phi,'r')
ylabel('phi')
legend("integrated", "normal");

subplot(3,1,3)
plot(time,l_i); hold on;grid on;
plot(time,l,'r')
ylabel('l')
legend("integrated", "normal");


doPlot = true;

if doPlot

    figure();
    plot3(X,Y,Z, '-k'); hold on;
    
    plot3(X_i,Y_i,Z_i, '-g'); hold on;
    % data Michele
    load('results100.mat');
    plot3(p(1,:),p(2,:),p(3,:),'-r');
        
    %%%%%
    plot3(X(1),Y(1),Z(1), 'ok', 'MarkerSize', 10);
    plot3(X(end),Y(end),Z(end), 'or', 'MarkerSize', 10);
    xlabel("x");
    ylabel("y");
    zlabel("z");
    title("Trajectory from black point to red");
    axis equal;
   
    legend("OCP","ISO","integrated");

    
    
%     % get the impulse force in time
%     fFun = ForceFun(time,Fun,Tth*Tf(1));
%     fFut = ForceFut(time,Fut,Tth*Tf(1));
%     
%     figure();
%     subplot(1,3,1);
%     plot(time,fFun);
%     title("Normal force Fun");
%     subplot(1,3,2);
%     plot(time,fFut);
%     title("Tangential force Fut");
%     subplot(1,3,3);
%     plot(time,Fr);
%     title("Winding force Fr");
% 
%     figure();
%     plot(time, sqrt(fFun.^2+fFut.^2)); hold on;
%     plot(time, ones(size(time,1),1)*Fun_max);
%     title("Actuation constraint");
% 
%     figure();
%     plot(time, fFut); hold on;
%     plot(time, fFun*0.8);
%     title("Friction constraint $F_{u,t} \leq \mu F_{u,n}$", 'Interpreter','Latex');
%     legend("Fut","mu*Fun");
% 
%     figure();
%     plot(time, KinEnergy);
%     title("Kinetic Energy");

end



function res = ForceFun(t,Fun,Tth)
   
  sigma = Tth/6;
  mu    = Tth/2;
  res = Fun.*exp(-(t-mu).^2./(2*sigma^2));
end

function res = ForceFut(t,Fut, Tth)
    sigma = Tth/6;
    mu    = Tth/2;
  res = Fut.*exp(-(t-mu).^2./(2*sigma^2));
end
