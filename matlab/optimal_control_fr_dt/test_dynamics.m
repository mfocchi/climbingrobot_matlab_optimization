close all 
clear all

global m g

state0 = [ 0.626            0            3          0.5          0.1            0];

m  = 5;
g = 9.81;

N_dyn = 10
Tf1 = 0.9;
Tf2 = 0.6;
Tf = Tf1 +Tf2;

dt_dyn = Tf/(2*N_dyn);
dt_dyn1 = Tf1 / (N_dyn-1);
dt_dyn2 = Tf2 / (N_dyn-1);
dt = 0.001;
int_steps = 10;

Fr = linspace(1,50,floor(Tf/dt));
Fr_rough = linspace(1,20, 2*N_dyn);
% 
% Fr = 130*ones(1,floor(Tf/dt));
% Fr_rough = 130*ones(1, 2*N_dyn);


%substep integraiton
%before obstacle
for i=1:N_dyn           
    i
    if (i>=2)     
        [states_rough_sub2(:,i), t_rough_sub2(i)] = integrate_dynamics(states_rough_sub2(:,i-1), t_rough_sub2(i-1), dt_dyn1/(int_steps-1), int_steps, Fr_rough(i-1)*ones(1,int_steps), 'rk4'); % keep Fr constant           
    
    t_rough_sub2(i)
    else
      states_rough_sub2(:,i) = state0;
      t_rough_sub2(i) = 0;      
    end    
    
end
%after obstacle
%substep integraiton
for i=N_dyn+1:2*N_dyn   
    i

    [states_rough_sub2(:,i), t_rough_sub2(i)] = integrate_dynamics(states_rough_sub2(:,i-1), t_rough_sub2(i-1), dt_dyn2/(int_steps-1), int_steps, Fr_rough(i-1)*ones(1,int_steps), 'rk4'); % keep Fr constant           
    t_rough_sub2(i)
end


% no substep integration
%before obstacle
[~,~,states_rough2(:, 1:N_dyn+1), t_rough2(1:N_dyn+1)] = integrate_dynamics(state0,0, dt_dyn1, N_dyn+1, Fr_rough(1:N_dyn), 'rk4');
% after obstacle
[~,~,states_rough2(: , N_dyn+2:2*N_dyn), t_rough2(N_dyn+2:2*N_dyn)] = integrate_dynamics(states_rough2(:,N_dyn+1),t_rough2(N_dyn+1), dt_dyn2, N_dyn-1, Fr_rough(N_dyn+1:2*N_dyn), 'rk4');


%as before with 2*N_dyn samples
[~,~,states_rough_total, t_rough_total] = integrate_dynamics(state0, 0,dt_dyn, 2*N_dyn, Fr_rough,'rk4');


%continuous
[~,~,states, t] = integrate_dynamics(state0,dt, dt, floor(Tf/dt), Fr); % N elements t start from dt 

figure
plot(t_rough_sub2, states_rough_sub2(1,:),'ko'); hold on
plot(t_rough2, states_rough2(1,:),'bo'); hold on
plot(t_rough_total, states_rough_total(1,:),'-go'); hold on
plot(t, states(1,:),'-r'); 
legend('rough2','rough2 sub','cont');

figure
plot(t_rough_sub2, states_rough_sub2(2,:),'ko'); hold on
plot(t_rough2, states_rough2(2,:),'bo'); hold on
%plot(t_rough_total, states_rough_total(2,:),'-go'); hold on
plot(t, states(2,:),'-r'); 
legend('rough2','rough2 sub','cont');


figure
plot(t_rough_sub2, states_rough_sub2(3,:),'ko'); hold on
plot(t_rough2, states_rough2(3,:),'bo'); hold on
%plot(t_rough_total, states_rough_total(3,:),'-go'); hold on
plot(t, states(3,:),'-r'); 
legend('rough2','rough2 sub','cont');



figure
plot(t_rough2, Fr_rough,'bo'); hold on
plot(t, Fr,'r'); hold on
