clear all
clc
close all


g = 9.81;

global delta_duration;
delta_duration =  0.05;

global friction_coefficient
friction_coefficient = 0.8;

global Fun0;       
Fun0 = 117;

global Fut0;
Fut0 = 93; 

global optK 
optK =   10.3738;

global l_0
l_0 = 3;

m = 5;   % Mass [kg]

global Tsim;
Tsim = 3;    % Simulation Time
dt = 0.001;
%tspan = [0:dt: Tsim];

load test.mat
tspan = time;

l_0 = 3;
theta0 =atan2(0.38, l_0);



%Initial Conditions
%x0 = [theta0; 0.; 0; 0; l_0; 0]; %[theta ;d/dt(theta) ;phi ;d/dt(phi) ;l ;d/dt(l)] 

x0 = [theta(1); thetad(1); phi(1); phid(1); l_0; ld(1)]; %[theta ;d/dt(theta) ;phi ;d/dt(phi) ;l ;d/dt(l)] 


global Fr_vec time

%%Simulation:
% define the stop function event
Opt    = odeset('Events', @stopFun);

%Solve differential equations var step 
[t, x] = ode23(@(t,x) diffEq(t,x,m,@Fr,@Fun, @Fut), tspan, x0, Opt); 

% x_log = x0;
% x = x0;
% for i=1:length(tspan)
%    xdot = diffEq(tspan(i),x,m,@Fr,@Fun, @Fut) 
%    x = x +xdot*dt;
%    x_log = [x_log x]; 
% end
% x = x_log';

%Solve differential equations fixed step (debug)
% [x] = ode4(@(t,x) diffEq(t,x,m,@Fr,@Fun, @Fut), tspan, x0);
% t = tspan;

theta_sim = x(:,1);
thetad_sim = x(:,2);
phi_sim = x(:,3);
phid_sim = x(:,4);
l_sim = x(:,5);
ld_sim = x(:,6);


%Coordinates
X = l_sim.*cos(phi_sim).*sin(theta_sim);
Y = l_sim.*sin(phi_sim).*sin(theta_sim);
Z = -l_sim.*cos(theta_sim);


fprintf('first touchdown is at : [%3.2f, %3.2f, %3.2f], for tf = %5.2f\n',X(end), Y(end), Z(end), t(end))

figure(1)
subplot(3,1,1)
plot(tspan, theta_sim); hold on;grid on;
plot(time, theta,'r');
ylabel('theta')
legend('sim', 'opt')

subplot(3,1,2)
plot(tspan, phi_sim); hold on;grid on;
plot(time, phi,'r');
ylabel('phi')
legend('sim', 'opt')


subplot(3,1,3)
plot(tspan, l_sim); hold on;grid on;
plot(time, l,'r');
ylabel('l')
legend('sim', 'opt')


% eval total energy
Etot_sim =  (m*l_sim.^2/2).*(thetad_sim.^2 + sin(theta_sim).^2 .*phid_sim.^2) +m.*ld_sim.^2/2 - m*g*l_sim.*cos(theta_sim) + optK*(l_sim-l_0).^2/2;
Ekin_sim= (m*l_sim.^2/2).*(thetad_sim.^2 + sin(theta_sim).^2 .*phid_sim.^2) +m.*ld_sim.^2/2;


figure(2)
plot(tspan, Ekin_sim);hold on;
plot(time, energy.Ekin,'r')

figure(3)
plot(tspan, Etot_sim);hold on;
plot(time, energy.Etot,'r')
 

% 3D plot Animation
figure(4)

title('3D Plot Animation');
xlabel('X ') ; ylabel('Y ') ; zlabel('Z ');

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


%     Vertical line
h(1) = plot3([0 0],[0 0],[0 -x0(5)],'k--');
%     Point fix
h(2) = plot3(0,0,0,'Marker','*','Color','k','MarkerSize',10);

%     drawing a wall at X = 0 
p1 = [0 min_y min_z];
p2 = [0 max_y min_z];
p3 = [0 max_y max_z];
p4 = [0 min_y max_z]; 
Xw = [p1(1) p2(1) p3(1) p4(1)];
Yw = [p1(2) p2(2) p3(2) p4(2)];
Zw = [p1(3) p2(3) p3(3) p4(3)];  
h(3) = fill3(Xw, Yw, Zw, 'g');
h(4) = animatedline('color','r', 'linewidth',3);
h(5) = animatedline('color','b', 'linewidth',3);
%Pendulum rod (red)
h(6) = animatedline('color','r', 'linewidth',3);
%Pendulum sphere (red)
h(7) = animatedline('Marker','o','Color','k','MarkerFaceColor','r','MarkerSize',10);

% Loop for animation
for i = 1:length(X)
    
    %Pendulum trajectory
    if t(i)<=delta_duration
        addpoints(h(4), X(i),Y(i),Z(i)); 

    else 
        addpoints(h(5)  ,X(i),Y(i),Z(i))
       
    end

    %Pendulum rod (red)
    %addpoints(h(6) , [0 X(i)],[0 Y(i)],[0 Z(i)]);

    %Pendulum sphere (red)
    %addpoints(h(7)  , X(i),Y(i),Z(i));
    
  
   
    drawnow;
    
    %frame = getframe(gcf);
        
    
%   writeVideo(v,frame);
    view(33,63);
    
end

% close(v);

%Frist component brake. Second component force.
function [fr] = Fr(t, x)
global optK l_0 Fr_vec time    
%   index = min(find(time(:)>=t(:)));
%   
%   fr = Fr_vec(index)
    %use interp to be precise
   fr = interp1(time(:),Fr_vec(:),t);
  %  fr = - optK*(x(5)-l_0);
end

function [f] = myGaussian(t, sigma, m)
    f = (1/(sqrt(2*pi*sigma^2)))*exp(-0.5*((t-m)/sigma)^2);
end


function [fun] = Fun(t)
    global delta_duration;
    global Fun0;
    if (t <= delta_duration)
        fun = Fun0;
    else
        fun = 0;
    end
    %fun = Fun0*myGaussian(t,delta_duration/6, delta_duration/2);
end

function [fut] = Fut(t)
     global delta_duration;
     global Fut0;
    if (t <= delta_duration)
        fut = Fut0;
    else
        fut = 0;
    end
    %fut = Fut0*myGaussian(t,delta_duration/6, delta_duration/2);
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
        
        value =  (X - 0.002 );
        
%         if (value < 0)
%             fprintf('Stopping\n');
%             fprintf('final target: \n ');
%             [X Y Z]
%         end
        %direction  =-1 Detect zero crossings in the negative direction only
        %direction  =0 Detect all zero crossings
        %direction  =1 Detect zero crossings in the positive direction only
        
        direction  = -1;
        isterminal = 1;   % Stop the integration

       
end

function [dxdt] = diffEq(t,x, m, Fr, Fun ,Fut)
%x = [theta, dottheta, phi, dotphi, l, dotl]
g = 9.8;   %Gravity   [m/s2]

%Retrieving states
theta = x(1);
dtheta = x(2);
phi = x(3);
dphi = x(4);
l = x(5);
dl = x(6);
% 
% ddtheta = -(2*dtheta*dl)/l + cos(theta)*sin(theta)*(dphi^2)-(g/l)*sin(theta)+Fun(t)/(m*l);
% ddphi = -2*(cos(theta)/sin(theta))*dphi*dtheta-(2/l)*dphi*dl+ Fut(t)/(m*l*sin(theta));
% ddl = l*(dtheta^2)+l*(sin(theta)^2)*dphi^2+g*cos(theta)+(1/m)*Fr(t, x);

ddtheta = -2/l*(dtheta*dl) + cos(theta)*sin(theta)*(dphi^2)-(g/l)*sin(theta) ;
ddphi = -2*(cos(theta)/sin(theta))*dphi*dtheta-(2/l)*dphi*dl ;
ddl = l*(dtheta^2)+l*(sin(theta)^2)*dphi^2+g*cos(theta)+(1/m)*Fr(t, x);

dxdt = [dtheta; ddtheta; dphi; ddphi; dl; ddl];

end

