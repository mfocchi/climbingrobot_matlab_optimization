clear all
clc
close all
warning off

global dmin; %Minimum distance from the mointain wall
dmin = 0.3770;

global dtol; %Tolerance for binding reaction of the wall
dtol = 1e-05;

global delta_duration;
delta_duration =  0.05;

global friction_coefficient
friction_coefficient = 0.8;

global Fun0;       
Fun0 = 117.1303;

global Fut0;
% this sets the tangential force always at the boundary of the cone (TODO
% set separate impulses connected with a friction cone constraint if you
% want to be more robust (i.e. stay more in the middle of the cone)
%Fut0 = friction_coefficient*Fun0
Fut0 = 93.7; 

global optK 
optK = 12.3110;

global l_0
l_0 = 3;

m = 5;   % Mass [kg]

global Tsim;
Tsim = 3;    % Simulation Time
tspan = [0:0.01: Tsim];


%Initial Conditions
x0 = [0.1260; 0; 0; 0; l_0; 0]; %[theta ;d/dt(theta) ;phi ;d/dt(phi) ;l ;d/dt(l)] 

%%Simulation:

Opt    = odeset('Events', @stopFun);
%Solve differential equations
[t,x] = ode23(@(t,x) diffEq(t,x,m,@Fr,@Fun, @Fut), tspan, x0,Opt); 




%Coordinates
X = x(:,5).*cos(x(:,3)).*sin(x(:,1));
Y = x(:,5).*sin(x(:,3)).*sin(x(:,1));
Z = -x(:,5).*cos(x(:,1));
% figure(1)
%plot3(X,Y,Z)
% comet3(X,Y,Z)      % Plot 3D slow motion curve using Comet3
% 
% 3D plot Animation

% Min-max axis
min_x = min(X)-3 ; 
max_x = max(X)+3 ;
min_y = min(Y)-3 ;
max_y = max(Y)+3 ;
min_z = min(Z)-2;
max_z = 2;

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
hold on ; grid on ; axis equal
set(gca,'CameraPosition',[10   35   10])
set(gca,'XLim',[min_x max_x])
set(gca,'YLim',[min_y max_y])
set(gca,'ZLim',[min_z max_z])
% set(gca,'XTickLabel',[],'YTickLabel',[],'ZTickLabel',[])

% Loop for animation
for i = 1:length(X)
    
    cla
%     Vertical line
    plot3([0 0],[0 0],[0 -x0(5)],'k--')
%     Point fix
    p = plot3(0,0,0,'Marker','*','Color','k','MarkerSize',10);
%     drawing a wall at X = 0 
    p1 = [0 min_y min_z];
    p2 = [0 max_y min_z];
    p3 = [0 max_y max_z];
    p4 = [0 min_y max_z]; 

    Xw = [p1(1) p2(1) p3(1) p4(1)];
    Yw = [p1(2) p2(2) p3(2) p4(2)];
    Zw = [p1(3) p2(3) p3(3) p4(3)];
    
    %Colors for wall: 'r','g','b','c','m','y','w','k', or an RGB row vector triple, [r g b]
    fill3(Xw, Yw, Zw, 'g');
%     Pendulum trajectory
    plot3(X(1:i),Y(1:i),Z(1:i),'b')
    title('3D Plot Animation')
    xlabel('X ') ; ylabel('Y ') ; zlabel('Z ')
%     Pendulum rod
    plot3([0 X(i)],[0 Y(i)],[0 Z(i)],'r')
%     Pendulum sphere
    plot3(X(i),Y(i),Z(i),'Marker','o','Color','k','MarkerFaceColor','r','MarkerSize',10);
    
%     Projections
%     plot3(min_x*ones(1,i),Y(1:i),Z(1:i),'g')
%     plot3(X(1:i),min_y*ones(1,i),Z(1:i),'g')
%     plot3(X(1:i),Y(1:i),min_z*ones(1,i),'g')
%     
    frame = getframe(gcf);
%   writeVideo(v,frame);
     
end

% close(v);

%%Functions used in the main program

%Frist component brake. Second component force.
function [fr] = Fr(t, x)
global Tsim optK l_0
%     if ((logical(t >= 0.10*Tsim)& logical(t <= 0.20*Tsim))),
%       %  fprintf('t = %f. Brakes off\n',t);
%         fr=[0,0];
%     else
%        % fprintf('t = %f. Brakes on\n',t);
%         fr = [1,0];
%     end
%fr = [1, 0];

    fr = - optK*(x(5)-l_0);
end

function [f] = myGaussian(t, sigma, m)
    f = (1/(sqrt(2*pi*sigma^2)))*exp(-0.5*((t-m)/sigma)^2);
end


function [fun] = Fun(t)
    global delta_duration;
    global Fun0;
%     if (t <= delta_duration)
%         fu = Fu0* 1/delta_duration;
%     else
%         fu = 0;
%     end
fun = Fun0*myGaussian(t,delta_duration/6, delta_duration/2);
end

function [fut] = Fut(t)
     global delta_duration;
     global Fut0;
%     if (t <= delta_duration)
%         ft = Ft0* 1/delta_duration;
%     else
%         ft = 0;
%     end
fut = Fut0*myGaussian(t,delta_duration/6, delta_duration/2);
end



function [value, isterminal, direction] = stopFun(t, x)

        theta = x(1);
        dtheta = x(2);
        phi = x(3);
        dphi = x(4);
        l = x(5);
        dl = x(6);
        
        
        X = l*cos(phi)*sin(theta);    
        Y = l*sin(phi)*sin(theta);
        Z = -l*cos(theta);
        
        value =  (X <= 0.001 );
        
        if (value)
            fprintf('Stopping\n');

            fprintf('final target: \n ');
            [X Y Z];
        end
        
                    isterminal = 1;   % Stop the integration
            direction  = 0;
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

%thmin = asin(dmin/l);
%thtol = asin(dtol/l);



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
        ddl = l*(dtheta^2)+l*(sin(theta)^2)*dphi^2+g*cos(theta)+(1/m)*Fr(t, x);


dxdt = [dtheta; ddtheta; dphi; ddphi; dl; ddl];

end

