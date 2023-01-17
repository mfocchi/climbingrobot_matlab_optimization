clear all
clc
close all

global delta_duration  Fleg  Fr1 Fr2  time DEBUG p_a1 p_a2

p_a1 = [0;-5;0];
p_a2 = [0;5;0];

g = 9.81;



DEBUG = false;
    
m = 7;   % Mass [kg]


delta_duration = 0.5;
dt = 0.001;
time =[0:dt:4];

Fleg = [100;0;0];
Fr1 = ones(length(time)) * -70;
Fr2 = ones(length(time)) * -30;


%Initial Conditions
% if DEBUG
%   x0 = [theta0; solution.thetad(1); phi0; solution.phid(1); l_0; solution.ld(1)]; %[theta ;d/dt(theta) ;phi ;d/dt(phi) ;l ;d/dt(l)] 
% else
%   x0 = [theta0; 0.;phi0; 0.; l_0; 0.]; %[theta ;d/dt(theta) ;phi ;d/dt(phi) ;l ;d/dt(l)] 
% end

p0 = [2; 3; -10];
l10 = norm(p0 - p_a1);
l20 = norm(p0 - p_a2);
l1d0 = 0.0;
l2d0 = 0.0;
J0 = [ (p0-p_a1)/norm(p0-p_a1) ,(p0-p_a2)/norm(p0-p_a2)];
dp0 = J0*[l1d0; l2d0];


%initial conditions
x0 = [p0; l10; l20; J0*[l1d0; l2d0]; l1d0; l2d0];

%%Simulation:
% define the stop function event
Opt    = odeset('Events', @stopFun);

%1 - Solve differential equations with variable step solver
[time_sim, x] = ode45(@(time_sim,x) diffEq(time_sim,x,m,@Fr1Fun,@Fr2Fun, @FlegFun), time, x0, Opt); 

X = x(:,1);
Y = x(:,2);
Z = x(:,3);


figure(1)
subplot(3,1,1)
plot(time_sim, X,'b'); hold on;grid on;
ylabel('X')
%legend('sim', 'opt')

subplot(3,1,2)
plot(time_sim, Y,'b'); hold on;grid on;
ylabel('Y')


subplot(3,1,3)
plot(time_sim, Z,'b'); hold on;grid on;
ylabel('Z')


% fprintf('original target is  : [%3.2f, %3.2f, %3.2f] \n',pf );
% fprintf('the Matlab touchdown is at : [%3.2f, %3.2f, %3.2f] , for tf = %5.2f\n',X(end), Y(end), Z(end), time_sim(end));
% if MICHELE_APPROACH
%     fprintf('expected optim target    : [%3.2f, %3.2f, %3.2f] \n',solution.achieved_target );
%     fprintf('with error : %3.2f\n',norm([solution.achieved_target(1);solution.achieved_target(2);solution.achieved_target(3)] - [X(end); Y(end);Z(end)]));
% end
% fprintf('with error : %3.2f\n',norm(pf - [X(end); Y(end);Z(end)]));
% %fprintf('with polar error : %3.2f\n',norm([theta_sim(end);phi_sim(end);l_sim(end)] - [solution.theta(end); solution.phi(end); solution.l(end)]));
% [Tf_gazebo, end_gazebo_index] = max(time_gazebo); % there are nans in the log
% fprintf('the Gazebo touchdown is at : [%3.2f, %3.2f, %3.2f] , for tf = %5.2f\n', traj_gazebo(1,end_gazebo_index),  traj_gazebo(2,end_gazebo_index), traj_gazebo(3,end_gazebo_index),Tf_gazebo);
% fprintf('with error : %3.2f\n',norm(pf - [traj_gazebo(1,end_gazebo_index);  traj_gazebo(2,end_gazebo_index); traj_gazebo(3,end_gazebo_index)]));
% 
% 
% % eval total energy sim
% Ekin_sim=   (m*l_sim.^2/2).*(thetad_sim.^2 + sin(theta_sim).^2 .*phid_sim.^2) +m.*ld_sim.^2/2;
% 
% 
% %compare with optim
% % figure(2)
% % plot(time_sim, Ekin_sim,'b.');hold on;grid on;
% % plot(time, solution.energy.Ekin,'r');
% % ylabel('Ekin') 
% % legend('sim', 'opt')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 3D plot Animation
figure(2)

title('Matlab Animation - simplified model');
xlabel('X ') ; ylabel('Y ') ; zlabel('Z ');

axis equal; hold on;
%anchor 1    
h(1) = plot3(p_a1(1),p_a1(2),p_a1(3),'.m', 'MarkerSize',40);grid on;hold on;
%anchor2    
h(2) = plot3(p_a2(1),p_a2(2),p_a2(3),'.y', 'MarkerSize',40);grid on;hold on;
%initial vel vector
h(3) = plot3([p0(1) p0(1)+dp0(1)],[p0(2) p0(2)+dp0(2)],[p0(3) p0(3)+dp0(3)],'g-');
%initial impulse
h(9) = plot3([p0(1) p0(1)+Fleg(1)/10],[p0(2) p0(2)+Fleg(2)/10],[p0(3) p0(3)++Fleg(3)/10],'g-');



%     Vertical line
h(4) = plot3([p_a1(1) p0(1)],[p_a1(2) p0(2)],[p_a1(3) p0(3)],'k-');
h(5) = plot3([p_a2(1) p0(1)],[p_a2(2) p0(2)],[p_a2(3) p0(3)],'k-');


min_x= 0;
max_x= 2*max(X);
min_z = min(Z);
max_z = 1;
min_y = -7;
max_y = 7;

%     drawing a wall at X = 0 
p1 = [0 min_y min_z];
p2 = [0 max_y min_z];
p3 = [0 max_y max_z];
p4 = [0 min_y max_z]; 
Xw = [p1(1) p2(1) p3(1) p4(1)];
Yw = [p1(2) p2(2) p3(2) p4(2)];
Zw = [p1(3) p2(3) p3(3) p4(3)];  
h(6) = fill3(Xw, Yw, Zw, 'b', 'FaceAlpha',.2  );

set(gca,'XLim',[min_x max_x])
set(gca,'YLim',[min_y max_y])
set(gca,'ZLim',[min_z max_z]) 
set(gca,'fontsize',30)

h(7) = animatedline('color','r', 'linewidth',3);
h(8) = animatedline('color','b', 'linewidth',3);

%Pendulum sphere (red)
% h(7) = animatedline('Marker','o','Color','k','MarkerFaceColor','r','MarkerSize',10);

view(60,27);

% Loop for animation
for i = 1:length(X)    
    %Pendulum trajectory
    if time_sim(i)<=delta_duration
        addpoints(h(7), X(i),Y(i),Z(i)); 

    else 
        addpoints(h(8)  ,X(i),Y(i),Z(i));
       
    end
    drawnow limitrate;
    pause(0.001);
 
end




function [fr1] = Fr1Fun(t)
global   Fr1 time
   idx = min(find(time>=t));
   fr1 = Fr1(idx);
end

function [fr2] = Fr2Fun(t)
global   Fr2 time
   idx = min(find(time>=t));
   fr2 = Fr2(idx);
end


function [fleg] = FlegFun(t)
global   Fleg  delta_duration 

    if (t <= delta_duration)
        fleg = Fleg;
    else
        fleg = [0;0;0];
    end
  
end


function [value, isterminal, direction] = stopFun(t, x)
        global time 
         
        %value =  (Z - target_height);
        value =  (time(end) - t);    
        %direction  =-1 Detect zero crossings of value in the negative direction only
        %direction  =0 Detect all zero crossings of value 
        %direction  =1 Detect zero crossings of value  in the positive direction only
        
        direction  = -1;
        isterminal = 1;   % Stop the integration

       
end

function [dxdt] = diffEq(t,x, m, Fr1, Fr2 ,Fleg)

    global DEBUG p_a1 p_a2

    %x = [theta, dottheta, phi, dotphi, l, dotl]
    g = 9.81;   %Gravity   [m/s2]

    %Retrieving states
    px = x(1);
    py = x(2);
    pz = x(3);
    l1 = x(4);
    l2 = x(5);
    dpx = x(6);
    dpy = x(7);
    dpz = x(8);
    dl1 = x(9);
    dl2 = x(10);

    p = [px;py;pz];
    dp = [dpx;dpy;dpz];

    % mass equation and rope constraints 
    A = [m*eye(3) ,    zeros(3,1)  , zeros(3,1),
         (p-p_a1)'  , -l1    ,   0,
         (p-p_a2)',   0    ,   -l2];
    
    if DEBUG % reset the state already

           b = [m*[0;0;-g]; -dp'*dp + dl1^2 ; -dp'*dp + dl2^2 ];     

    else
       
        J = [ (p-p_a1)/norm(p-p_a1) ,(p-p_a2)/norm(p-p_a2)];
        b = [m*[0;0;-g] + J*[Fr1(t);Fr2(t)] + Fleg(t); -dp'*dp + dl1^2 ; -dp'*dp + dl2^2 ];     
    end

    y = inv(A)*b;
    dxdt = [dpx; dpy; dpz; dl1; dl2; y];


end

