clear all
clc
close all

global dmin; %Minimum distance from the mointain wall
dmin = 1;

global dtol; %Tolerance for binding reaction of the wall
dtol = 1e-05;

global delta_duration;
delta_duration = 2e-1;

global friction_coefficient
friction_coefficient = 0.8;

global Fun0;       
Fun0 = 80;

global Fut0;
% this sets the tangential force always at the boundary of the cone (TODO
% set separate impulses connected with a friction cone constraint if you
% want to be more robust (i.e. stay more in the middle of the cone)
Fut0 = friction_coefficient*Fun0

m = 10;   % Mass [kg]

global Tsim;
Tsim = 15;    % Simulation Time
tspan = [0:0.001: Tsim];

global l;
l = 10;
g = 9.8;
%Initial Conditions
x0 = [0.1; 0.1; 0.3; 0.05]; %[theta ;d/dt(theta) ;phi ;d/dt(phi) ]
theta0 = x0(1);
dtheta0 = x0(2);
phi0 = x0(3);
dphi0 = x0(4);

A= [0, 1, 0, 0;
    cos(2*theta0)*(dphi0^2)-(g/l)*cos(theta0), 0, 0, sin(2*theta0)*dphi0;
    0, 0, 0, 1;
    (2/(sin(theta0)^2))*dtheta0*dphi0, -2*(cos(theta0)/sin(theta0))*dphi0, 0, -2*(cos(theta0)/sin(theta0))*dtheta0];
    

D = eig(A);
Tperiod = (abs(imag(D(4)))/(2*pi))^-1 % is very similar to 2*pi*sqrt(l/g)


%%Simulation:
%Solve differential equations
[t,x] = ode23(@(t,x) diffEq(t,x,m,l), tspan, x0); 




%Coordinates
X = l.*cos(x(:,3)).*sin(x(:,1));
Y = l*sin(x(:,3)).*sin(x(:,1));
Z = -l*cos(x(:,1));


%Solve differential equations
[t,xl] = ode23(@(t,x) diffEqLin(t,x,x0, m,l), tspan, x0); 
% xl1 = zeros(size(xl));
% for i = 1:max(size(xl))
%     xl1(i,:) = xl(i,:)+x0';
% end


%Coordinates
Xl = l.*cos(xl(:,3)).*sin(xl(:,1));
Yl = l*sin(xl(:,3)).*sin(xl(:,1));
Zl = -l*cos(xl(:,1));

subplot(4,1,1);
plot(t,x(:,1))
hold
plot(t,xl(:,1))
legend('Nonlinear', 'linearised');
ylabel('theta')

subplot(4,1,2)
plot(t,x(:,2))
hold
plot(t,xl(:,2))
ylabel('thetad')
legend('Nonlinear', 'linearised');

subplot(4,1,3);
plot(t,x(:,3))
hold
plot(t,xl(:,3))
legend('Nonlinear', 'linearised');
ylabel('phi')


subplot(4,1,4)
plot(t,x(:,4))
hold
plot(t,xl(:,4))
legend('Nonlinear', 'linearised');
ylabel('phid')

figure
subplot(3,1,1)
plot(t,X)
hold
plot(t,Xl)
legend('Nonlinear', 'linearised');
ylabel('X')


subplot(3,1,2)
plot(t,Y)
hold
plot(t,Yl)
legend('Nonlinear', 'linearised');
ylabel('Y')

subplot(3,1,3)
plot(t,Z)
hold
plot(t,Zl)
legend('Nonlinear', 'linearised');
ylabel('Z')

% figure(1)
%plot3(X,Y,Z)
% comet3(X,Y,Z)      % Plot 3D slow motion curve using Comet3
% 
% % 3D plot Animation
% 
% Min-max axis
% min_x = min(X)-3 ; 
% max_x = max(X)+3 ;
% min_y = min(Y)-3 ;
% max_y = max(Y)+3 ;
% min_z = min(Z)-2;
% max_z = 0;
% 
% figure(2)
% set(gcf,'Position',[50 50 1280 720]) % YouTube: 720p
% set(gcf,'Position',[50 50 854 480]) % YouTube: 480p
% set(gcf,'Position',[50 50 640 640]) % Social
% 
% Create and open video writer object
% v = VideoWriter('spherical_pendulum.mp4');
% v.Quality = 100;
% open(v);
%     
% hold on ; grid on ; axis equal
% set(gca,'CameraPosition',[42.0101   30.8293   16.2256])
% set(gca,'XLim',[min_x max_x])
% set(gca,'YLim',[min_y max_y])
% set(gca,'ZLim',[min_z max_z])
% set(gca,'XTickLabel',[],'YTickLabel',[],'ZTickLabel',[])
% 
% Loop for animation
% for i = 1:length(X)
%     
%     cla
%     Vertical line
%     plot3([0 0],[0 0],[0 -x0(5)],'k--')
%     Point fix
%     p = plot3(0,0,0,'Marker','*','Color','k','MarkerSize',10);
%     Pendulum trajectory
%     plot3(X(1:i),Y(1:i),Z(1:i),'b')
%     Pendulum rod
%     plot3([0 X(i)],[0 Y(i)],[0 Z(i)],'r')
%     Pendulum sphere
%     plot3(X(i),Y(i),Z(i),'Marker','o','Color','k','MarkerFaceColor','r','MarkerSize',10);
%     Projections
%     plot3(min_x*ones(1,i),Y(1:i),Z(1:i),'g')
%     plot3(X(1:i),min_y*ones(1,i),Z(1:i),'g')
%     plot3(X(1:i),Y(1:i),min_z*ones(1,i),'g')
%     
%     frame = getframe(gcf);
%   writeVideo(v,frame);
%     
% end
% 
% close(v);

%%Functions used in the main program

function [dxdt] = diffEq(t,x, m, l)
global dmin;
global dtol;

%x = [theta, dottheta, phi, dotphi, l, dotl]

g = 9.8;   %Gravity   [m/s2]

%Retrieving states
theta = x(1);
dtheta = x(2);
phi = x(3);
dphi = x(4);
X = l*cos(phi)*sin(theta);
%thmin = asin(dmin/l);
%thtol = asin(dtol/l);
% if (X < dmin )
%         fprintf('Parking\n');
%         ddtheta = 0;
%         dtheta = 0;
%         ddphi = 0;
%         dphi = 0;
% else
    
%     fprintf('Flying\n');
%     %Flight dynamics
%     ddtheta = -2*dtheta*dl/l + cos(theta)*sin(theta)*(dphi^2)- (g/l)*sin(theta);
%     ddphi = -2*(cos(theta)/sin(theta))*dphi*dtheta-(2/l)*dphi*dl;
%     ddl = l*(dphi^2)+l*(sin(theta)^2)*dphi+g*cos(theta)+(1/m)*FR(2);
% else
% %     Fun(t)
% %     if (abs(Fun(t)) > 1e-6)
%         fprintf('Taking off\n');
%         %Take off dynamicx
fprintf('Flying\n');
        ddtheta =  cos(theta)*sin(theta)*(dphi^2)-(g/l)*sin(theta);
        ddphi = -2*(cos(theta)/sin(theta))*dphi*dtheta;
%%end
dxdt = [dtheta; ddtheta; dphi; ddphi];

end


%%Functions used in the main program

function [dxdt] = diffEqLin(t,x,x0, m, l)
global dmin;
global dtol;

%x = [theta, dottheta, phi, dotphi, l, dotl]

g = 9.8;   %Gravity   [m/s2]

%Retrieving states
theta = x(1);
dtheta = x(2);
phi = x(3);
dphi = x(4);

%Retrieving states
theta0 = x0(1);
dtheta0 = x0(2);
phi0 = x0(3);
dphi0 = x0(4);


X = l*cos(phi)*sin(theta);
%thmin = asin(dmin/l);
%thtol = asin(dtol/l);
% if (X < dmin )
%         fprintf('Parking\n');
%         ddtheta = 0;
%         dtheta = 0;
%         ddphi = 0;
%         dphi = 0;
% else
%     
%     fprintf('Flying\n');
%     %Flight dynamics
%     ddtheta = -2*dtheta*dl/l + cos(theta)*sin(theta)*(dphi^2)- (g/l)*sin(theta);
%     ddphi = -2*(cos(theta)/sin(theta))*dphi*dtheta-(2/l)*dphi*dl;
%     ddl = l*(dphi^2)+l*(sin(theta)^2)*dphi+g*cos(theta)+(1/m)*FR(2);
% else
% %     Fun(t)
% %     if (abs(Fun(t)) > 1e-6)
%         fprintf('Taking off\n');
%         %Take off dynamicx
fprintf('Flying\n');

ddtheta =  [cos(2*theta0)*(dphi0^2)-(g/l)*cos(theta0), 0, 0, sin(2*theta0)*dphi0]*[theta; dtheta; phi; dphi];
ddphi = [(2/(sin(theta0)^2))*dtheta0*dphi0, -2*(cos(theta0)/sin(theta0))*dphi0, 0, -2*(cos(theta0)/sin(theta0))*dtheta0]*[theta; dtheta; phi; dphi];

%end
dxdt = [dtheta; ddtheta; dphi; ddphi];

end


