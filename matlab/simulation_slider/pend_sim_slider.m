clear all
clc
close all

global delta_duration   Fun  Fut Fr Fs l_0  time   MICHELE_APPROACH DEBUG
g = 9.81;

MICHELE_APPROACH =true;
OBSTACLE = false;
DEBUG = false;
% 
% load ('test_gazebo.mat')
% load ('test_optim_marco.mat')
% time_gazebo = time
% traj_gazebo = solution.p
% Fun = solution.Fun;
% Fut = solution.Fut; 
% Fr = solution.Fr;
% time = solution.time;  
%pf =  cast(pf,"like", p0); % cast to double in case was save as an int in gazebo



m = 5.08;   % Mass [kg]
m_s = 2;   % Mass [kg]

delta_duration = 0.05;
p0 = [0.1;0; -6];
anchor_distance = 5;
 
dt = 0.01;
time =[0:dt:8];
Fut = 0;
Fun = 300;
Fr = ones(length(time)) * -40;
Fs = ones(length(time)) * 10;

p0 = p0(:);
[theta0, phi0, l_0] = computePolarVariables(p0);
s0 =0;

%Initial Conditions
x0 = [s0; 0.; theta0; 0.; phi0; 0.;  l_0; 0.]; %[s, sdot, theta ;d/dt(theta) ;phi ;d/dt(phi) ;l ;d/dt(l)] 

%%Simulation:
% define the stop function event
Opt    = odeset('Events', @stopFun);

%1 - Solve differential equations with variable step solver
[time_sim, x] = ode45(@(time_sim,x) diffEq(time_sim,x,m,m_s, @FsFun, @FrFun,@FunFun, @FutFun), time, x0, Opt); 

s_sim = x(:,1);
sd_sim = x(:,2);
theta_sim = x(:,3);
thetad_sim = x(:,4);
phi_sim = x(:,5);
phid_sim = x(:,6);
l_sim = x(:,7);
ld_sim = x(:,8);


%Coordinates
X = l_sim.*cos(phi_sim).*sin(theta_sim);
Y = s_sim + l_sim.*sin(phi_sim).*sin(theta_sim);
Z = -l_sim.*cos(theta_sim);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 3D plot Animation
figure(1)

title('Matlab Animation - simplified model');
xlabel('X ') ; ylabel('Y ') ; zlabel('Z ');

% Min-max axis
 % Min-max axis
min_x= 0;
max_x= 2*max(X);
min_z = min(Z);
max_z = 1;
min_y =  2*min(Y);
max_y = 2*max(Y);


hold on ; grid on ; axis equal
set(gca,'CameraPosition',[10   35   10])
set(gca,'XLim',[min_x max_x])
set(gca,'YLim',[min_y max_y])
set(gca,'ZLim',[min_z max_z])

%     Vertical line
h(1) = plot3([0 p0(1)],[0 p0(2)],[0 p0(3)],'k-');
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

%h(8) = plot3(pf(1),pf(2),pf(3),'Marker','.','Color','r','MarkerSize',50);
if OBSTACLE 
    cone(0,1.5,0);
end


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
%plot( time, solution.p(1,:),'r', 'linewidth', 4);hold on;
plot( time_sim, s_sim,'b'); 
%plot( time_gazebo, traj_gazebo(1,:),'k'); 
ylabel('$s_y$','interpreter','latex')
set(ha(1),'XtickLabel',[])
xlim([0, time_sim(end)])


ha(2) = axes('position',[four_xgraph four_y2 four_w small_h]);
%plot( time, solution.p(1,:),'r', 'linewidth', 4);hold on;
plot( time_sim, X,'b'); 
%plot( time_gazebo, traj_gazebo(1,:),'k'); 
ylabel('$p_\mathrm{x} [\mathrm{m}]$','interpreter','latex')
set(ha(1),'XtickLabel',[])
xlim([0, time_sim(end)])

lgd=legend({'$\mathrm{opt}$', ...
    '$\mathrm{sim~mat}$', ...
    '$\mathrm{sim~gaz}$',},...
        'Location','southeast',...
        'interpreter','latex',...
        'Orientation','horizontal');
lgd.FontSize = 25;

ha(3) = axes('position',[four_xgraph four_y3 four_w small_h]);
%plot(time, solution.p(2,:),'r', 'linewidth', 4);hold on;
plot(time_sim, Y,'b'); 
%plot(time_gazebo, traj_gazebo(2,:),'k'); 
ylabel('$p_\mathrm{y} [\mathrm{m}]$','interpreter','latex')
set(ha(2),'XtickLabel',[])
xlim([0, time_sim(end)])

ha(4) = axes('position',[four_xgraph four_y4 four_w small_h]);
%plot( time, solution.p(3,:),'r', 'linewidth', 4);hold on;
plot(time_sim, Z,'b'); 
%plot(time_gazebo, traj_gazebo(3,:),'k'); 
ylabel('$p_\mathrm{z} [\mathrm{m}]$','interpreter','latex')
xlabel('t $[\mathrm{s}]$','interpreter','latex')
xlim([0, time_sim(end)])



function [fs] = FsFun(t)
global   Fs time
   idx = min(find(time>=t));
   fs = Fs(idx);
end


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
        s = x(1);
        sd = x(2);
        theta = x(3);
        dtheta = x(4);
        phi = x(5);
        dphi = x(6);
        l = x(7);
        dl = x(8);
           
        X = l*cos(phi)*sin(theta);    
        Y = s + l*sin(phi)*sin(theta);
        Z = -l*cos(theta);
  
        value = X;%stop when gets to wall
        %value =  (Z - target_height);
        %value =  (time(end) - t);    
        %direction  =-1 Detect zero crossings of value in the negative direction only
        %direction  =0 Detect all zero crossings of value 
        %direction  =1 Detect zero crossings of value  in the positive direction only
        
        direction  = -1;
        isterminal = 1;   % Stop the integration

       
end

function [dxdt] = diffEq(t,x, m, m_s, Fs, Fr, Fun ,Fut)

    global DEBUG

    %x = [theta, dottheta, phi, dotphi, l, dotl]
    g = 9.81;   %Gravity   [m/s2]

    %Retrieving states
    s = x(1);
    ds = x(2);
    theta = x(3);
    dtheta = x(4);
    phi = x(5);
    dphi = x(6);
    l = x(7);
    dl = x(8);

            %1
%     ( m +m_s)* dds +sin(phi)*sin(theta)*m*ddl +l*m*cos(phi)* sin(theta)*ddphi +  +cos(theta)*sin(phi)*l*m*ddtheta +
%             2*l*m*cos(phi)*cos(theta)*dphi*dtheta -2*dl*m*cos(phi)*sin(theta)*dphi
%             +sin(phi)*sin(theta)*(-l*m*dtheta^2 -l*m*dphi^2 )
%             +2*cos(theta)*sin(phi)* dl*m*dtheta =
%         Fs(t) + Fut(t)* cos(phi) + Fun(t)*cos(theta)* sin(phi) + Fr(t) *sin(phi)* sin(theta);
%      %2          
%     ddtheta + (2*dtheta*dl)/l - cos(theta)*sin(theta)*(dphi^2)  + (g/l)*sin(theta) +  1/l*cos(theta)*sin(phi)*sdd  = Fun(t)/(m*l);
%     
%     %3
%     ddphi + 2*(cos(theta)/sin(theta))*dphi*dtheta  + (2/l)*dphi*dl +1/(l*sin(theta))*cos(phi)*dds =Fut(t)/(m*l*sin(theta));
%     %4
%     
%     ddl-l*(dtheta^2) -l*(sin(theta)^2)*dphi^2 -g*cos(theta) +sin(phi)*sin(theta) * dds = (1/m)*Fr(t);
   
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    a11 = m + m_s;                   %dds       
    a12 = cos(theta)*sin(phi)*l*m ;       %ddtheta
    a13 = l*m*cos(phi)* sin(theta);        %ddphi
    a14=sin(phi)*sin(theta)*m; %ddl
    
    F1 = Fs(t) + Fut(t)* cos(phi) + Fun(t)*cos(theta)* sin(phi) + Fr(t) *sin(phi)* sin(theta); 
    b1 = 2*l*m*cos(phi)*cos(theta)*dphi*dtheta -2*dl*m*cos(phi)*sin(theta)*dphi +sin(phi)*sin(theta)*(-l*m*dtheta^2 -l*m*dphi^2 )    +2*cos(theta)*sin(phi)* dl*m*dtheta;
            
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    a21 = 1/l*cos(theta)*sin(phi);%dds       
    a22 =1;%ddtheta
    a23 = 0; %ddphi
    a24 = 0;%ddl
    
    b2 = (2*dtheta*dl)/l - cos(theta)*sin(theta)*(dphi^2)  + (g/l)*sin(theta);
    F2 =  Fun(t)/(m*l);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    a31 = 1/(l*sin(theta))*cos(phi); %dds       
    a32 =0;%ddtheta
    a33 = 1; %ddphi
    a34 = 0;%ddl
    
    b3 = + 2*(cos(theta)/sin(theta))*dphi*dtheta  + (2/l)*dphi*dl ;
    F3 = Fut(t)/(m*l*sin(theta));
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    a41 =     sin(phi)*sin(theta);%dds       
    a42 =0;%ddtheta
    a43 = 0; %ddphi
    a44 = 1;%ddl
    
    b4= -l*(dtheta^2) -l*(sin(theta)^2)*dphi^2 -g*cos(theta);    
    F4 = (1/m)*Fr(t);
    
    A  = [a11, a12, a13, a14;
         a21, a22, a23, a24;
         a31, a32, a33, a34;
         a41, a42, a43, a44];
     
     b = [F1-b1;     
          F2-b2;
          F3-b3;
          F4-b4];
    
    y = inv(A)*b;
                     
    %dxdt = [ds; dds; dtheta; ddtheta; dphi; ddphi; dl; ddl];
       
    dxdt = [ ds;y(1);dtheta; y(2); dphi; y(3); dl; y(4)];

end

