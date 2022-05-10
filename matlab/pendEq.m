clear all
clc
close all

global dmin; %Minimum distance from the mointain wall
dmin = 0.5;

global dtol; %Tolerance for binding reaction of the wall
dtol = 1e-05;

global delta_duration;
delta_duration = 2e-1;

global Fu0;       
Fu0 = 80;

global Ft0;
Ft0 = 60;

m = 10;   % Mass [kg]

global Tsim;
Tsim = 10;    % Simulation Time
tspan = [0:0.01: Tsim];



%Initial Conditions
x0 = [asin(dmin/5); 0; 0; 0; 5; 0]; %[theta ;d/dt(theta) ;phi ;d/dt(phi) ;l ;d/dt(l)] 

%%Simulation:

%Solve differential equations
[t,x] = ode23(@(t,x) diffEq(t,x,m,@Fr,@Fu, @Ft), tspan, x0); 

%Coordinates
X = x(:,5).*cos(x(:,3)).*sin(x(:,1));
Y = x(:,5).*sin(x(:,3)).*sin(x(:,1));
Z = -x(:,5).*cos(x(:,1));
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

%Frist component brake. Second component force.
function [fr] = Fr(t)
global Tsim;
if ((logical(t >= 0.10*Tsim)& logical(t <= 0.20*Tsim))),
  %  fprintf('t = %f. Brakes off\n',t);
    fr=[0,0];
else
   % fprintf('t = %f. Brakes on\n',t);
    fr = [1,0];
end
%fr = [1, 0];
end
function [f] = myGaussian(t, sigma, m)
    f = (1/(sqrt(2*pi*sigma^2)))*exp(-0.5*((t-m)/sigma)^2);
end
function [fu] = Fu(t)
    global delta_duration;
    global Fu0;
%     if (t <= delta_duration)
%         fu = Fu0* 1/delta_duration;
%     else
%         fu = 0;
%     end
fu = Fu0*myGaussian(t,delta_duration/6, delta_duration/2);
end

function [ft] = Ft(t)
     global delta_duration;
     global Ft0;
%     if (t <= delta_duration)
%         ft = Ft0* 1/delta_duration;
%     else
%         ft = 0;
%     end
ft = Ft0*myGaussian(t,delta_duration/6, delta_duration/2);
end

function [dxdt] = diffEq(t,x, m, Fr, Fun ,Fut)
global dmin;
global dtol;

%x = [theta, dottheta, phi, dotphi, l, dotl]

g = 9.8;   %Gravity   [m/s2]

%Retrieving states
theta = x(1);
dtheta = x(2);
phi = x(3);
dphi = x(4);
l = x(5);
dl = x(6);
X = l*cos(phi)*sin(theta);
%thmin = asin(dmin/l);
%thtol = asin(dtol/l);
FR = Fr(t);
if (X < dmin )
        fprintf('Parking\n');
        ddtheta = 0;
        dtheta = 0;
        ddphi = 0;
        dphi = 0;
        ddl = 0;
        dl = 0;
else
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
        ddtheta = -(2*dtheta*dl)/l + cos(theta)*sin(theta)*(dphi^2)-(g/l)*sin(theta)+Fun(t)/(m*l);
        ddphi = -2*(cos(theta)/sin(theta))*dphi*dtheta-(2/l)*dphi*dl+ Fut(t)/(m*l*sin(theta));
        ddl = l*(dtheta^2)+l*(sin(theta)^2)*dphi^2+g*cos(theta)+(1/m)*FR(2);
end
%Brake on?
if FR(1) > 0.99,
    ddl = 0;
    dl =0;
 %   fprintf('Brakes on\n');
else
  %  fprintf('Brakes off\n');
end
dxdt = [dtheta; ddtheta; dphi; ddphi; dl; ddl];
if(FR(1)<0.99)
display(dxdt)
end

end

