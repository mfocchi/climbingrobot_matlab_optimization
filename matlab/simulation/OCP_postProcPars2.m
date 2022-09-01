clc;
clear all;
close all;

% additional data for plotting
m     = 5;
Tth   = 0.1;
sigma = Tth/6;
mu    = Tth/2;
Fun_max = 600;
%%%%%%%%%%%%%%

T = table2array(readtable('Rocciatore_OCP_result_last','NumHeaderLines',20)); % 21
%T = table2array(readtable('../pinsEnergy/generated_code/data/Rocciatore_OCP_result','NumHeaderLines',21)); % 21
%T = table2array(readtable('../pins/generated_code/data/Rocciatore_OCP_result_NOT_CONVERGED','NumHeaderLines',27));


%1.i_segment	2.zeta	3.lagrange_target	4.inequality_penalties	5.penalties	6.control_penalties	7.Fr	8.Fr_cell	9.theta	10.phi	11.psi	12.l	13.omega	14.r	15.T	16.lambda1__xo	17.lambda2__xo	18.lambda3__xo	19.lambda4__xo	20.lambda5__xo	21.lambda6__xo	22.lambda7__xo	23.theta_D	24.phi_D	25.psi_D	26.l_D	27.omega_D	28.r_D	29.T_D	30.mu0_D	31.mu1_D	32.mu2_D	33.mu3_D	34.mu4_D	35.mu5_D	36.mu6_D	37.FrControl	38.T_positive	39.Actuation	40.Friction

% 21
% 1.i_segment	2.zeta	3.lagrange_target	4.inequality_penalties	5.penalties	6.control_penalties	7.Fr	8.Fr_cell	
% 9.theta	10.phi	11.psi	12.l	13.omega	14.r	15.T	16.Fut	17.Fun	
% 18.lambda1__xo	19.lambda2__xo	20.lambda3__xo	21.lambda4__xo	22.lambda5__xo	23.lambda6__xo	24.lambda7__xo	
% 25.lambda8__xo	26.lambda9__xo	
% 27. theta_D	28.phi_D	29.psi_D	30.l_D	31.omega_D	32.r_D	33.T_D	34.Fut_D	35.Fun_D	
% 36.mu0_D	37.mu1_D	38.mu2_D	39.mu3_D	40.mu4_D	41.mu5_D	42.mu6_D	43.mu7_D	44.mu8_D	45.FrControl	46.T_positive	Actuation
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

% get the impulse force
fFun = ForceFun(zeta,Fun,sigma,mu,m);
fFut = ForceFut(zeta,Fut,sigma,mu,m);




pathL = 0;
npts = size(zeta,1);
s = zeros(1,npts);

intEkin = 0;
dt = 0.01;
for i=1:npts-1
    pathL = pathL + sqrt((X(i+1)-X(i))^2+(Y(i+1)-Y(i))^2+(Z(i+1)-Z(i))^2);
    s(i+1) = pathL;
    intEkin = intEkin + KinEnergy(i)*dt;    
end
pathL
pp0 = [X(1),Y(1),Z(1)]';
pp1 = [X(2),Y(2),Z(2)]';
ppN = [X(npts),Y(npts),Z(npts)]';
t0 = (pp1-pp0)/norm(pp1-pp0);
n0 = findNormal(X,Y,Z);
b0 = cross(t0,n0);





doPlot = true;

if doPlot

    figure();
    plot3(X,Y,Z, '-k'); hold on;
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
   
    legend("OCP","ISO");

    figure();
    subplot(1,3,1);
    plot(time,fFun);
    title("Normal force Fun");
    subplot(1,3,2);
    plot(time,fFut);
    title("Tangential force Fut");
    subplot(1,3,3);
    plot(time,Fr);
    title("Winding force Fr");

    figure();
    plot(time, sqrt(fFun.^2+fFut.^2)); hold on;
    plot(time, ones(size(time,1),1)*Fun_max);
    title("Actuation constraint");

    figure();
    plot(time, fFut); hold on;
    plot(time, fFun*0.8);
    title("Friction constraint $F_{u,t} \leq \mu F_{u,n}$", 'Interpreter','Latex');
    legend("Fut","mu*Fun");

    figure();
    plot(time, KinEnergy);
    title("Kinetic Energy");

end



function res = ForceFun(zeta,Fun,sigma,mu,m)
  res = Fun/sqrt(2*pi*sigma^2).*exp(-(zeta-mu).^2./(2*sigma^2));
end

function res = ForceFut(zeta,Fut,sigma,mu,m)
  res = Fut/sqrt(2*pi*sigma^2).*exp(-(zeta-mu).^2./(2*sigma^2));
end

function n0 = findNormal(x,y,z)
    P1x = x(1);
    P1y = y(1);
    P1z = z(1);
    P2x = x(2);
    P2y = y(2);
    P2z = z(2);
    P3x = x(3);
    P3y = y(3);
    P3z = z(3);
    D21x = P2x-P1x;
    D21y = P2y-P1y;
    D21z = P2z-P1z;
    D31x = P3x-P1x;
    D31y = P3y-P1y;
    D31z = P3z-P1z;

    F2 = 1/2*(D21x^2+D21y^2+D21z^2);
    F3 = 1/2*(D31x^2+D31y^2+D31z^2);

    M23xy = D21x*D31y-D21y*D31x;
    M23yz = D21y*D31z-D21z*D31y;
    M23xz = D21z*D31x-D21x*D31z;

    F23x = F2*D31x-F3*D21x;
    F23y = F2*D31y-F3*D21y;
    F23z = F2*D31z-F3*D21z;

    Cx = P1x+(M23xy*F23y-M23xz*F23z)/(M23xy^2+M23yz^2+M23xz^2);
    Cy = P1y+(M23yz*F23z-M23xy*F23x)/(M23xy^2+M23yz^2+M23xz^2);
    Cz = P1z+(M23xz*F23x-M23yz*F23y)/(M23xy^2+M23yz^2+M23xz^2);
    C = [Cx,Cy,Cz]';
    p0 = [x(1),y(1),z(1)]';
    n0 = -(p0-C)/norm(p0-C);
end
