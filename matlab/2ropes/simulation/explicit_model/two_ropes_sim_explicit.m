clear all
clc
close all

global delta_duration  Fleg  Fr1 Fr2  time  optim_time OPTIM  p_a1 p_a2 b   g m

%cd to actual dir
filePath = matlab.desktop.editor.getActiveFilename;
pathparts = strsplit(filePath,filesep);
dirpath= pathparts(1:end-1);
actual_dir =  strjoin(dirpath,"/");
cd(actual_dir);


OPTIM = true;
%possible settings
test_type='normal'; %IRIM
%test_type='obstacle_avoidance';  
%test_type='landing_test';


load('tests/test_irim_gazebo.mat');

if OPTIM %inputs from optim
    if strcmp(test_type, 'obstacle_avoidance')
        load ('tests/test_matlab2obstacle.mat');
        m = 5.08;   % Mass [kg]
    elseif strcmp(test_type, 'landing_test')  
        load ('tests/test_matlab2landingClearance.mat');
        m = 15.07 ;   % Mass [kg]    
    else    	
        load ('tests/test_matlab2.mat');
        m = 5.08;   % Mass [kg]
    end           
            
    %load ('test_matlab_cpp.mat');    
    %load ('test_matlab2landingClearance.mat');
    Tf = solution.Tf; 
    dt = 0.001;
    time = [0:dt:Tf];
    Fleg = solution.Fleg;
    optim_time = solution.time;
    Fr1 = solution.Fr_l; %resample creates oscillations!
    Fr2 = solution.Fr_r;
    delta_duration = solution.T_th;
    force_scaling = 100;
    p0 = solution.p(:,1);
    
else %fixed inputs
    Tf = 4;
    dt = 0.001;
    time =[0:dt:Tf];
    delta_duration = 0.05;
    %inputs
    Fleg = [200;0;0];
    Fr1 = ones(1, length(time)) * -40;
    Fr2 = ones(1, length(time)) * -30;
    force_scaling = 20;
    %jump params
    p0 = [0.0; 2.5; -6]; % there is singularity for px = 0!
    m = 5.08;   % Mass [kg]
end     


%WORLD FRAME ATTACHED TO ANCHOR 1
anchor_distance = 5;
b = anchor_distance;
p_a1 = [0;0;0];
p_a2 = [0;anchor_distance;0];
g = 9.81;


l10 = norm(p0 - p_a1);
l20 = norm(p0 - p_a2);
l1d0 = 0.0;
l2d0 = 0.0;
J0 = computeJacobian(p0);
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


% as an alternative you can call directly integrate dynamics
%Ndyn = length(Fr1);
% [~,~,x, time_sim] = integrate_dynamics(x0, 0, Tf/(N_dyn-1), N_dyn,Fr1, Fr2, Fleg,'rk4');
 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 3D plot Animation
figure(3)

%title('Matlab Animation - reduced order model');
%xlabel('X ') ; 
ylabel('Y ') ; 
zlabel('Z ');

axis equal; hold on;
%anchor 1    
h(1) = plot3(p_a1(1),p_a1(2),p_a1(3),'.m', 'MarkerSize',40);grid on;hold on;
%anchor2    
h(2) = plot3(p_a2(1),p_a2(2),p_a2(3),'.y', 'MarkerSize',40);grid on;hold on;

%initial leg impulse (red)
h(9) = plot3([p0(1) p0(1)+Fleg(1)/force_scaling],[p0(2) p0(2)+Fleg(2)/force_scaling],[p0(3) p0(3)+Fleg(3)/force_scaling],'r','Linewidth',4);

% rope forces
Fr1_vec =  (p0 - p_a1)/norm(p0 - p_a1)*Fr1(1);
Fr2_vec =  (p0 - p_a2)/norm(p0 - p_a2)*Fr2(1);
%Fr1 (red ) (forces are positive if they accelerate the mass
h(10) = plot3([p0(1) p0(1)+Fr1_vec(1)/force_scaling],[p0(2) p0(2)+Fr1_vec(2)/force_scaling],[p0(3) p0(3)+Fr1_vec(3)/force_scaling],'r','Linewidth',4);
%Fr2 (red ) 
h(11) = plot3([p0(1) p0(1)+Fr2_vec(1)/force_scaling],[p0(2) p0(2)+Fr2_vec(2)/force_scaling],[p0(3) p0(3)+Fr2_vec(3)/force_scaling],'r','Linewidth',4);


%plot world reference frame
Tt = [eye(3), [0;0;0];
    zeros(1,3) 1];
tt = hgtransform('Matrix', Tt);
h(12) = triad('Parent',tt, 'linewidth', 6);

%  plot rope lines (black)
h(4) = plot3([p_a1(1) p0(1)],[p_a1(2) p0(2)],[p_a1(3) p0(3)],'k-');
h(5) = plot3([p_a2(1) p0(1)],[p_a2(2) p0(2)],[p_a2(3) p0(3)],'k-');

min_x= 0;
max_x= 2*max(X);
min_z = min(Z)-2;
max_z = 1;
min_y = -2;
max_y = min_y+anchor_distance+4;

%  drawing a wall at X = 0 
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

h(7) = animatedline('color','g', 'linewidth',3);
h(8) = animatedline('color','b', 'linewidth',3);

%Pendulum sphere (red)
% h(7) = animatedline('Marker','o','Color','k','MarkerFaceColor','r','MarkerSize',10);

if strcmp(test_type, 'obstacle_avoidance')   
    h(9) = plot_ellipsoid([-0.5, 3,-4.5],1.5, 1.5, 0.866,  min_z, max_z, min_y,max_y);
end
view(60,27);

% Loop for animation
for i = 1:length(X)    
    %Pendulum trajectory position of the pendulum green during push blue
    %during flight
    if time_sim(i)<=delta_duration
        addpoints(h(7), X(i),Y(i),Z(i)); 

    else 
        addpoints(h(8)  ,X(i),Y(i),Z(i));
       
    end
    drawnow limitrate;
    pause(0.001);
 
end

% plot target point (red)
h(13) = plot3(X(end),Y(end), Z(end),'.r', 'MarkerSize',40);
axis equal

view(113,61); 
%save the plot
set(gcf, 'Paperunits' , 'centimeters')
set(gcf, 'PaperSize', [20 21]);
set(gcf, 'PaperPosition', [0 0 20 21]);
print(gcf, '-dpdf',strcat('irim_optimal_trajectory.pdf'),'-painters')



% OTHER PLOTS
if OPTIM
    figure(1)
    subplot(3,1,1)
    plot(time_sim, X,'b'); hold on;grid on;
    plot(solution.time, solution.p(1,:),'ro');
    ylabel('X')
    legend('sim', 'opt')

    subplot(3,1,2)
    plot(time_sim, Y,'b'); hold on;grid on;
    plot(solution.time, solution.p(2,:),'ro');
    ylabel('Y')

    subplot(3,1,3)
    plot(time_sim, Z,'b'); hold on;grid on;
    plot(solution.time, solution.p(3,:),'ro');
    ylabel('Z')
    figure(5)
    plot(optim_time, Fr1,'r-o'); hold on; grid on;
    plot(optim_time, Fr2,'b-o'); hold on; grid on;
end


matlab_final_point = [X(end);Y(end);Z(end)];
[Tf_gazebo, end_gazebo_index] = max(time_gazebo); % there are nans in the log
gazebo_final_point = [actual_com(1,end_gazebo_index);actual_com(2,end_gazebo_index);actual_com(3,end_gazebo_index)];
jump_length  = norm(p0 -solution.path_length);
if OPTIM
    fprintf('expected Optim target [%3.4f, %3.4f, %3.4f] \n', solution.achieved_target)
    fprintf('matlab touch down point [%3.4f, %3.4f, %3.4f] \n', matlab_final_point)
    fprintf('error norm matlab: %3.4f perc: %3.4f\n',norm(matlab_final_point - solution.achieved_target), norm(matlab_final_point - solution.achieved_target)/jump_length*100)
end
fprintf('Gazebo touch down point [%3.4f, %3.4f, %3.4f] \n', gazebo_final_point)
fprintf('error norm gazebo :  %3.4f perc: %3.4f \n',norm(gazebo_final_point - solution.achieved_target), norm(gazebo_final_point - solution.achieved_target)/jump_length*100)
fprintf('Touchdown at s t [%3.4f] \n', time_sim(end))






%%%%PLOTS for paper
%%
loadFigOptions
figure(2)
ha(1) = axes('position',[four_xgraph four_y1 four_w small_h]);
plot( solution.time, solution.p(1,:),'r', 'linewidth', 4);hold on;
plot( time_sim, X,'b'); 
plot( time_gazebo, actual_com(1,:),'k'); 
ylabel('$p_\mathrm{x} [\mathrm{m}]$','interpreter','latex')
set(ha(1),'XtickLabel',[])
xlim([0,  solution.time(end)])

lgd=legend({'$\mathrm{opt}$', ...
    '$\mathrm{sim~mat}$', ...
    '$\mathrm{sim~gaz}$',},...
        'Location','northwest',...
        'interpreter','latex',...
        'Orientation','horizontal');
lgd.FontSize = 25;

ha(2) = axes('position',[four_xgraph four_y2 four_w small_h]);
plot( solution.time, solution.p(2,:),'r', 'linewidth', 4);hold on;
plot(time_sim, Y,'b'); 
plot(time_gazebo, actual_com(2,:),'k'); 
ylabel('$p_\mathrm{y} [\mathrm{m}]$','interpreter','latex')
set(ha(2),'XtickLabel',[])
xlim([0,  solution.time(end)])

ha(3) = axes('position',[four_xgraph four_y3 four_w small_h]);
plot(  solution.time, solution.p(3,:),'r', 'linewidth', 4);hold on;
plot(time_sim, Z,'b'); 
plot(time_gazebo, actual_com(3,:),'k'); 
ylabel('$p_\mathrm{z} [\mathrm{m}]$','interpreter','latex')
set(ha(3),'XtickLabel',[])
xlim([0,  solution.time(end)])

ha(4) = axes('position',[four_xgraph four_y4 four_w small_h]);
plot( solution.time, Fr_l0,'r');hold on; hold on;
plot( solution.time, Fr_r0,'b');hold on;
plot( solution.time, -Fr_max*ones(1,length(Fr_r0)),'k');hold on;
ylabel('$F_r [\mathrm{N}]$','interpreter','latex')
xlabel('t $[\mathrm{s}]$','interpreter','latex')
xlim([0,  solution.time(end)])

lgd=legend({'left','right', 'max'},...
        'Location','southwest',...
        'interpreter','latex',...
        'Orientation','horizontal');
lgd.FontSize = 25;

% save the plot
set(gcf, 'Paperunits' , 'centimeters')
set(gcf, 'PaperSize', [30 20]);
set(gcf, 'PaperPosition', [0 0 30 20]);
print(gcf, '-dpdf','irim_validation.pdf','-painters')



% this is needed because the intergration time t might be different from
% the discretization of the input Fr
function [fr1] = Fr1Fun(t)
global   Fr1 time optim_time OPTIM

   if OPTIM
       t_ = optim_time;
   else 
       t_ = time;
   end
   idx = min(find(t_>t))-1;
   fr1 = Fr1(idx);
end

% this is needed because the intergration time t might be different from
% the discretization of the input Fr
function [fr2] = Fr2Fun(t)
global   Fr2 time optim_time OPTIM

   if OPTIM
       t_ = optim_time;
   else 
       t_ = time;
   end
   idx = min(find(t_>t))-1;
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



function [value, isterminal, direction] = stopFun(t, x )
         global OPTIM 
        
        
         
        if OPTIM
            %value =  optim_time(end) - t;
            value =1; % stops at the end of time
        else
            value = x(1);%stop when gets to wall
        end
      

        %value =  (Z - target_height);
        %value =  (time(end) - t);    
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
       
        J =  computeJacobian(p);
        b = [m*[0;0;-g] + J*[Fr1(t);Fr2(t)] + Fleg(t); -dp'*dp + dl1^2 ; -dp'*dp + dl2^2 ];     
    end

    y = inv(A)*b;
    dxdt = [dpx; dpy; dpz; dl1; dl2; y];


end

