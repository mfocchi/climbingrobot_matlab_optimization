clc;
close all;
clear all;

p0 = [0.2244; 0 ; -8];

tt = table2array(readtable('reachable')); 

mu = 0.7;


n = size(tt,1);
figure();
hold on;
m = 5; %kg

x_vec=[]; 
y_vec =[];
z_vec = []; 
% convergence =[] 
Tf_vec =[]; 
Lagrange_vec =[];
Mayer_vec = [];
% Fun = []; 
% Fut = [];
Ekin0_vec = [];
Ekinf_vec = [];
Ehoist_vec =[];
E_comsumed_vec =[];

% extract data
for i=1:n    
    if (tt(i,4)==1)   
        x = tt(i,1);
        y = tt(i,2);
        z = tt(i,3);        
        Tf = tt(i,5);
        Lagrange = tt(i,6); %int( 0.0005*Fr^2*l_d^2) dt 
        Mayer = tt(i,7);
        x_vec = [x_vec; x];        
        y_vec = [y_vec; y];
        z_vec = [z_vec; z];
        Tf_vec = [Tf_vec;Tf];
        Lagrange_vec = [Lagrange_vec; Lagrange];
        Mayer_vec = [Mayer_vec; Mayer];
        
        % we compute the impulse work with the LIFTOFF kinetic energy (from rough
        % approx of velocity computed in Tth
        %dt = Tf / 500;
        %T_th =  Tf*0.0646;
        %dp = diff([x y z])./dt;
        %Ekin0 = 0.5*m*dp(floor(T_th/dt),:)*dp(floor(T_th/dt),:);
        Ekinf = (Mayer -Tf)*100; % (Mayer - Tf )*100 because Mayer = 1*Tf + 0.01 *Ekinf
        Ehoist = Lagrange/0.0005;
        %Ekin0_vec = [Ekin0_vec Ekin0]
        Ekinf_vec = [Ekinf_vec Ekinf];
        Ehoist_vec = [Ehoist_vec Ehoist];
        %not used: int(Fut * e_t +Fun * e_n + Fr*e_r).dot(dx)
        %E_comsumed_vec = [E_comsumed_vec (Ehoist +Ekin0 )];
    end
end

plot_reachable_region(p0, x_vec, y_vec, z_vec, mu, sqrt(Ehoist_vec), '$\int \Vert F_r dp \Vert [J]$', [5, 45]);
view (90,90);
set(gcf, 'Paperunits' , 'centimeters')
set(gcf, 'PaperSize', [40 25]);
set(gcf, 'PaperPosition', [0 0 40 25]);
print(gcf, '-dpdf',strcat('../../paper/matlab/reachable.pdf'),'-painters')


% plot_reachable_region(p0, x_vec, y_vec, z_vec, mu, Ekinf_vec, '$K_f$');
% set(gcf, 'Paperunits' , 'centimeters')
% set(gcf, 'PaperSize', [20 30]);
% set(gcf, 'PaperPosition', [0 0 20 30]);
% print(gcf, '-dpdf',strcat('../../paper/matlab/reachable2.pdf'),'-painters')

