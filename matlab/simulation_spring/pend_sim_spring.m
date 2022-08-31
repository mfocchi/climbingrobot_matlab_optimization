clear all
clc
close all
g = 9.81;

load ('../optimal_control/test.mat')

 
global delta_duration friction_coefficient Fun0  Fut0 l_0 optK Tsim Fr_vec time target_height DEBUG

m = 5;   % Mass [kg]
DEBUG = 0;

delta_duration =  T_th;
friction_coefficient= mu; 
Fun0 = solution.Fun;
Fut0 = solution.Fut; 
optK = solution.K;
time = solution.time;
target_height = pf(3);
[theta0, phi0, l_0] = computePolarVariables(p0);

dt = 0.001;
Tsim = 3.;

if DEBUG
    %to debug reset the initial states
    Fr_vec = solution.Fr_vec;
   
    tspan = solution.time;
    x0 = [solution.theta(1); solution.thetad(1); solution.phi(1); solution.phid(1); l_0; solution.ld(1)]; %[theta ;d/dt(theta) ;phi ;d/dt(phi) ;l ;d/dt(l)] 

else
    %Initial Conditions
    x0 = [theta0; 0.; phi0; 0.; l_0; 0.]; %[theta ;d/dt(theta) ;phi ;d/dt(phi) ;l ;d/dt(l)] 
    tspan = [0:dt: Tsim]; % use the event function to stop!
end


%%Simulation:
% define the stop function event
Opt    = odeset('Events', @stopFun);

%1 - Solve differential equations with variable step solver
[time_sim, x] = ode23(@(time_sim,x) diffEq(time_sim,x,m,@Fr,@Fun, @Fut), tspan, x0, Opt); 

% 2- Euler solution
% x_log = x0;
% x = x0;
% for i=1:length(tspan)
%    xdot = diffEq(tspan(i),x,m,@Fr,@Fun, @Fut) 
%    x = x +xdot*dt;
%    x_log = [x_log x]; 
% end
% x = x_log';

% 3- Solve differential equations with  fixed step solver(debug)
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

fprintf('first touchdown is at : [%3.2f, %3.2f, %3.2f], for tf = %5.2f\n',X(end), Y(end), Z(end), time_sim(end));

figure(1)
subplot(3,1,1)
plot(time_sim, theta_sim,'b.'); hold on;grid on;
plot(time, solution.theta,'r');
ylabel('theta')
legend('sim', 'opt')

subplot(3,1,2)
plot(time_sim, phi_sim,'b.'); hold on;grid on;
plot(time, solution.phi,'r');
ylabel('phi')
legend('sim', 'opt')


subplot(3,1,3)
plot(time_sim, l_sim,'b.'); hold on;grid on;
plot(time, solution.l,'r');
ylabel('l')
legend('sim', 'opt')

%derivatives
% figure(1)
% subplot(3,1,1)
% plot(time_sim, thetad_sim,'b.'); hold on;grid on;
% plot(time, solution.thetad,'r');
% ylabel('thetad')
% legend('sim', 'opt')
% 
% subplot(3,1,2)
% plot(time_sim, phid_sim,'b.'); hold on;grid on;
% plot(time, solution.phid,'r');
% ylabel('phid')
% legend('sim', 'opt')
% 
% 
% subplot(3,1,3)
% plot(time_sim, ld_sim,'b.'); hold on;grid on;
% plot(time, solution.ld,'r');
% ylabel('ld')
% legend('sim', 'opt')



% eval total energy sim
Etot_sim =  (m*l_sim.^2/2).*(thetad_sim.^2 + sin(theta_sim).^2 .*phid_sim.^2) +m.*ld_sim.^2/2 -m*g*l_sim.*cos(theta_sim) + optK*(l_sim-l_0).^2/2;
Ekin_sim=   (m*l_sim.^2/2).*(thetad_sim.^2 + sin(theta_sim).^2 .*phid_sim.^2) +m.*ld_sim.^2/2;


%compare with optim
figure(2)
plot(time_sim, Ekin_sim,'b.');hold on;grid on;
plot(time, solution.energy.Ekin,'r');
ylabel('Ekin') 

figure(3)
plot(time_sim, Etot_sim,'b.');hold on; grid on;
plot(time, solution.energy.Etot','r');
ylabel('Etot') 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
h(1) = plot3([0 p0(1)],[0 p0(2)],[0 p0(3)],'k--');
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
h(3) = fill3(Xw, Yw, Zw, 'b', 'FaceAlpha',.5  );

h(4) = animatedline('color','r', 'linewidth',3);
h(5) = animatedline('color','b', 'linewidth',3);
%Pendulum rod (red)
h(6) = animatedline('color','r', 'linewidth',3);
%Pendulum sphere (red)
h(7) = animatedline('Marker','o','Color','k','MarkerFaceColor','r','MarkerSize',10);

h(8) = plot3(pf(1),pf(2),pf(3),'Marker','.','Color','r','MarkerSize',50);

 view(33,63);
% Loop for animation
for i = 1:length(X)
    
    %Pendulum trajectory
    if time_sim(i)<=delta_duration
        addpoints(h(4), X(i),Y(i),Z(i)); 

    else 
        addpoints(h(5)  ,X(i),Y(i),Z(i))
       
    end

    %Pendulum rod (red)
    %addpoints(h(6) , [0 X(i)],[0 Y(i)],[0 Z(i)]);

    %Pendulum sphere (red)
    %addpoints(h(7)  , X(i),Y(i),Z(i));
       
    drawnow limitrate;
    pause(0.001);
    %frame = getframe(gcf);    
%   writeVideo(v,frame);   
    
end

% close(v);




function [fr] = Fr(t, x)
global optK l_0 Fr_vec time DEBUG    

    if DEBUG
    % use simulated Fr
    %   index = min(find(time(:)>=t(:)));
    %   
    %   fr = Fr_vec(index)
        %use interp to be precise
       fr = interp1(time(:),Fr_vec(:),t);
    else
       fr = - optK*(x(5)-l_0);
    end

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
        global target_height time delta_duration
        theta = x(1);
        dtheta = x(2);
        phi = x(3);
        dphi = x(4);
        l = x(5);
        dl = x(6);
        
        
        X = l*cos(phi)*sin(theta);    
        Y = l*sin(phi)*sin(theta);
        Z = -l*cos(theta);
        
        %value =  (Z - target_height);
        value =  (time(end) - t);

    
        %direction  =-1 Detect zero crossings of value in the negative direction only
        %direction  =0 Detect all zero crossings of value 
        %direction  =1 Detect zero crossings of value  in the positive direction only
        
        direction  = -1;
        isterminal = 1;   % Stop the integration

       
end

function [dxdt] = diffEq(t,x, m, Fr, Fun ,Fut)
global DEBUG

%x = [theta, dottheta, phi, dotphi, l, dotl]
g = 9.81;   %Gravity   [m/s2]

%Retrieving states
theta = x(1);
dtheta = x(2);
phi = x(3);
dphi = x(4);
l = x(5);
dl = x(6);

if DEBUG % reset the state already
  ddtheta = -2/l*(dtheta*dl) + cos(theta)*sin(theta)*(dphi^2)-(g/l)*sin(theta) ;
  ddphi = -2*(cos(theta)/sin(theta))*dphi*dtheta-(2/l)*dphi*dl ;
  ddl = l*(dtheta^2)+l*(sin(theta)^2)*dphi^2+g*cos(theta)+(1/m)*Fr(t, x);
else
  ddtheta = -(2*dtheta*dl)/l + cos(theta)*sin(theta)*(dphi^2)-(g/l)*sin(theta)+Fun(t)/(m*l);
  ddphi = -2*(cos(theta)/sin(theta))*dphi*dtheta-(2/l)*dphi*dl+ Fut(t)/(m*l*sin(theta));
  ddl = l*(dtheta^2)+l*(sin(theta)^2)*dphi^2+g*cos(theta)+(1/m)*Fr(t, x);
end

dxdt = [dtheta; ddtheta; dphi; ddphi; dl; ddl];

end

