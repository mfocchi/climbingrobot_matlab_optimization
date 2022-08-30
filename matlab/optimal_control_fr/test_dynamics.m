state0 = [ 0.626            0            3          0.5          0.1            0];
K = 10;
N_dyn = 20
Tf = 1.5
dt_dyn = Tf / N_dyn;
dt = 0.001;

Fr = 130*ones(1,floor(Tf/dt));
Fr_rough = 130*ones(1, N_dyn);


[~,~,states, t] = integrate_dynamics(state0,0, dt, floor(Tf/dt), Fr);
[~,~,states_rough, t_rough] = integrate_dynamics(state0, 0,dt_dyn, N_dyn, Fr,'rk4');

figure
plot(t_rough, states_rough(1,:),'-bo'); hold on
plot(t, states(1,:),'-r'); 
figure
plot(t_rough, states_rough(2,:),'-bo'); hold on
plot(t, states(2,:),'-r');
figure
plot(t_rough, states_rough(3,:),'-bo'); hold on
plot(t, states(3,:),'-r');