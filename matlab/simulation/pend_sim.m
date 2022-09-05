clear all
clc
close all

global delta_duration friction_coefficient Fun  Fut Fr l_0      time   MICHELE_APPROACH DEBUG
g = 9.81;


MICHELE_APPROACH = true;
DEBUG = false;

if MICHELE_APPROACH
    load ('test_gazebo.mat')
    %load ('../optimal_control_fr/test_michele.mat')
else    
    load ('test_marco.mat')
end
    
m = 5;   % Mass [kg]

delta_duration =  T_th;
friction_coefficient= mu; 
Fun = solution.Fun;
Fut = solution.Fut; 
Fr = solution.Fr;
time = solution.time;
p0 = p0(:);
pf =  cast(pf,"like", p0); % cast to double in case was save as an int in gazebo
pf = pf(:);
[theta0, phi0, l_0] = computePolarVariables(p0);


dt = 0.001;

%Initial Conditions
if DEBUG
  x0 = [theta0; solution.thetad(1); phi0; solution.phid(1); l_0; solution.ld(1)]; %[theta ;d/dt(theta) ;phi ;d/dt(phi) ;l ;d/dt(l)] 
else
  x0 = [theta0; 0.;phi0; 0.; l_0; 0.]; %[theta ;d/dt(theta) ;phi ;d/dt(phi) ;l ;d/dt(l)] 
end

%%Simulation:
% define the stop function event
Opt    = odeset('Events', @stopFun);

%1 - Solve differential equations with variable step solver
[time_sim, x] = ode45(@(time_sim,x) diffEq(time_sim,x,m,@FrFun,@FunFun, @FutFun), time, x0, Opt); 

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

fprintf('the touchdown is at : [%3.2f, %3.2f, %3.2f] , for tf = %5.2f\n',X(end), Y(end), Z(end), time_sim(end));
fprintf('expected optim target    : [%3.2f, %3.2f, %3.2f] \n',solution.achieved_target );
fprintf('with error : %3.2f\n',norm([solution.achieved_target(1);solution.achieved_target(2);solution.achieved_target(3)] - [X(end); Y(end);Z(end)]));
fprintf('original target     : [%3.2f, %3.2f, %3.2f] \n',pf );
fprintf('with error : %3.2f\n',norm(pf - [X(end); Y(end);Z(end)]));



% eval total energy sim
Ekin_sim=   (m*l_sim.^2/2).*(thetad_sim.^2 + sin(theta_sim).^2 .*phid_sim.^2) +m.*ld_sim.^2/2;


%compare with optim
% figure(2)
% plot(time_sim, Ekin_sim,'b.');hold on;grid on;
% plot(time, solution.energy.Ekin,'r');
% ylabel('Ekin') 
% legend('sim', 'opt')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 3D plot Animation
figure(1)

title('3D Plot Animation');
xlabel('X ') ; ylabel('Y ') ; zlabel('Z ');

% Min-max axis
 % Min-max axis
min_x = min(min(X), pf(1))-3 ; 
max_x = max(max(X), pf(1))+3 ;
min_y = min(min(Y), pf(2))-3 ;
max_y = max(max(Y), pf(2)) +3 ;
min_z = min(min(Z, pf(3,:)))-4;
max_z = 2;
max_z = 2;

hold on ; grid on ; axis equal
set(gca,'CameraPosition',[10   35   10])
set(gca,'XLim',[min_x max_x])
set(gca,'YLim',[min_y max_y])
set(gca,'ZLim',[min_z max_z])

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
      
    drawnow limitrate;
    pause(0.001);

end


%%%%PLOTS for paper
%%
loadFigOptions
figure(2)
ha(1) = axes('position',[four_xgraph four_y1 four_w small_h]);
plot( time, solution.p(1,:),'r');hold on;
plot( time_sim, X,'b'); 
plot( time_gazebo, traj_gazebo(1,:),'k');
ylabel('$p_\mathrm{x} [\mathrm{m}]$','interpreter','latex')
set(ha(1),'XtickLabel',[])
xlim([0, time(end)])

lgd=legend({'$\mathrm{opt}$', ...
    '$\mathrm{sim~mat}$', ...
    '$\mathrm{sim~gaz}$',},...
        'Location','northwest',...
        'interpreter','latex',...
        'Orientation','horizontal');
lgd.FontSize = 25;

ha(2) = axes('position',[four_xgraph four_y2 four_w small_h]);
plot(time, solution.p(2,:),'r');hold on;
plot(time_sim, Y,'b'); 
plot(time_gazebo, traj_gazebo(2,:),'k');
ylabel('$p_\mathrm{y} [\mathrm{m}]$','interpreter','latex')
set(ha(2),'XtickLabel',[])
xlim([0, time(end)])

ha(3) = axes('position',[four_xgraph four_y3 four_w small_h]);
plot( time, solution.p(3,:),'r');hold on;
plot(time_sim, Z,'b'); 
plot(time_gazebo, traj_gazebo(3,:),'k');
ylabel('$p_\mathrm{z} [\mathrm{m}]$','interpreter','latex')
set(ha(3),'XtickLabel',[])
xlim([0, time(end)])

ha(4) = axes('position',[four_xgraph four_y4 four_w small_h]);
plot(time, solution.Fr,'r');hold on;
ylabel('$F_r [\mathrm{N}]$','interpreter','latex')
xlabel('t $[\mathrm{s}]$','interpreter','latex')
xlim([0, time(end)])

% save the plot
set(gcf, 'Paperunits' , 'centimeters')
set(gcf, 'PaperSize', [45 19]);
set(gcf, 'PaperPosition', [0 0 45 19]);
print(gcf, '-dpdf',strcat('../../paper/matlab/sim_results.pdf'),'-painters')


% 
% figure(1)
% subplot(3,1,1)
% plot(time_sim, theta_sim,'b.'); hold on;grid on;
% plot(time, solution.theta,'r');
% ylabel('theta')
% legend('sim', 'opt')
% 
% subplot(3,1,2)
% plot(time_sim, phi_sim,'b.'); hold on;grid on;
% plot(time, solution.phi,'r');
% ylabel('phi')
% legend('sim', 'opt')
% 
% 
% subplot(3,1,3)
% plot(time_sim, l_sim,'b.'); hold on;grid on;
% plot(time, solution.l,'r');
% ylabel('l')
% legend('sim', 'opt')

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


















function [fr] = FrFun(t)
global   Fr time
   idx = min(find(time>=t));
   fr = Fr(idx);
end


function [fun] = FunFun(t)
global   Fun time delta_duration MICHELE_APPROACH
    if MICHELE_APPROACH
        if (t <= delta_duration)
            fun = Fun;
        else
            fun = 0;
        end
    else
       idx = min(find(time>=t));
       fun = Fun(idx);   
    end
end


function [fut] = FutFun(t)
global   Fut time delta_duration MICHELE_APPROACH
    if MICHELE_APPROACH
        if (t <= delta_duration)
            fut = Fut;
        else
            fut = 0;
        end
    else
       idx = min(find(time>=t));
       fut = Fut(idx);   
    end
end

function [value, isterminal, direction] = stopFun(t, x)
        global time 
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
    ddl = l*(dtheta^2)+l*(sin(theta)^2)*dphi^2+g*cos(theta)+(1/m)*Fr(t);
else
    ddtheta = -(2*dtheta*dl)/l + cos(theta)*sin(theta)*(dphi^2)-(g/l)*sin(theta)+Fun(t)/(m*l);
    ddphi = -2*(cos(theta)/sin(theta))*dphi*dtheta  -(2/l)*dphi*dl+ Fut(t)/(m*l*sin(theta));
    ddl = l*(dtheta^2)+l*(sin(theta)^2)*dphi^2+g*cos(theta)+(1/m)*Fr(t);
end

dxdt = [dtheta; ddtheta; dphi; ddphi; dl; ddl];

end

